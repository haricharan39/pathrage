"""HTTP + WebSocket hub for the 3D viewer (aiohttp, runs in its own thread).

One port serves the static page (/), its assets (/static/...) and the data socket (/ws).

Two delivery classes, because the browser keeps ONE big GPU buffer for the map:
  * reliable  - map deltas, trajectory additions, 'cleared': every message must arrive, in order.
  * latest    - live cloud, Tier-1 points, robot state: only the newest matters, so a slow
                browser gets the newest frame instead of a growing backlog.
A client whose reliable backlog grows too large is simply re-synced with a fresh snapshot.
"""
import asyncio
import collections
import json
import pathlib
import threading

from aiohttp import WSMsgType, web

MAX_BACKLOG = 3000


class _Client:
    def __init__(self, ws):
        self.ws = ws
        self.reliable = collections.deque()
        self.latest = {}
        self.wake = asyncio.Event()
        self.alive = True


class ViewerServer:
    def __init__(self, web_dir, host="127.0.0.1", port=8080, snapshot_provider=None,
                 command_handler=None, logger=None):
        self.web_dir = pathlib.Path(web_dir)
        self.host, self.port = host, int(port)
        self._snapshot = snapshot_provider or (lambda: [])
        self._on_command = command_handler or (lambda cmd: None)
        self._log = logger
        self._loop = None
        self._thread = None
        self._runner = None
        self._clients = set()
        self._ready = threading.Event()
        self._error = None

    # ------------------------------------------------------------------ lifecycle
    def start(self, timeout=10.0):
        self._thread = threading.Thread(target=self._run, name="viewer-http", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
            raise RuntimeError("viewer server did not start in time")
        if self._error is not None:
            raise RuntimeError(f"viewer server failed to start on {self.host}:{self.port}: "
                               f"{self._error}")

    def stop(self):
        if self._loop is None:
            return
        fut = asyncio.run_coroutine_threadsafe(self._shutdown(), self._loop)
        try:
            fut.result(timeout=5)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)

    @property
    def client_count(self):
        return len(self._clients)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except Exception as exc:  # port in use, etc.
            self._error = exc
            self._ready.set()
            return
        self._ready.set()
        self._loop.run_forever()

    async def _serve(self):
        @web.middleware
        async def no_cache(request, handler):
            resp = await handler(request)
            resp.headers["Cache-Control"] = "no-cache"
            return resp

        app = web.Application(middlewares=[no_cache])
        app.router.add_get("/", self._index)
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_static("/static", self.web_dir, show_index=False)
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self.host, self.port)
        await site.start()

    async def _shutdown(self):
        for c in list(self._clients):
            c.alive = False
            await c.ws.close()
        if self._runner:
            await self._runner.cleanup()

    # ------------------------------------------------------------------ handlers
    async def _index(self, request):
        return web.FileResponse(self.web_dir / "index.html")

    async def _ws_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=1 << 20)
        await ws.prepare(request)
        client = _Client(ws)
        self._clients.add(client)  # register first so no delta is missed during the snapshot
        self._prime(client)
        sender = asyncio.ensure_future(self._sender(client))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        self._on_command(json.loads(msg.data))
                    except Exception as exc:  # malformed command must never kill the socket
                        if self._log:
                            self._log(f"bad viewer command: {exc}")
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            client.alive = False
            client.wake.set()
            sender.cancel()
            self._clients.discard(client)
        return ws

    def _prime(self, client):
        """Queue a full snapshot ahead of any deltas that arrived since registration.

        Map writes are index-addressed, so a delta that overlaps the snapshot is harmless.
        """
        frames = list(self._snapshot())
        client.reliable.extendleft(reversed(frames))
        client.wake.set()

    async def _sender(self, client):
        try:
            while client.alive:
                await client.wake.wait()
                client.wake.clear()
                while client.reliable:
                    await self._send(client, client.reliable.popleft())
                for key in list(client.latest):
                    await self._send(client, client.latest.pop(key))
        except (asyncio.CancelledError, ConnectionResetError):
            pass
        except Exception as exc:
            client.alive = False
            if self._log:
                self._log(f"viewer client dropped: {exc}")

    @staticmethod
    async def _send(client, payload):
        if isinstance(payload, (bytes, bytearray)):
            await client.ws.send_bytes(payload)
        else:
            await client.ws.send_str(payload)

    # ------------------------------------------------------------------ thread-safe API
    def broadcast_reliable(self, payload):
        if self._loop is None or not self._clients:
            return
        self._loop.call_soon_threadsafe(self._push_reliable, payload)

    def broadcast_latest(self, key, payload):
        if self._loop is None or not self._clients:
            return
        self._loop.call_soon_threadsafe(self._push_latest, key, payload)

    def _push_reliable(self, payload):
        for c in list(self._clients):
            if len(c.reliable) > MAX_BACKLOG:
                c.reliable.clear()
                self._prime(c)
            c.reliable.append(payload)
            c.wake.set()

    def _push_latest(self, key, payload):
        for c in self._clients:
            c.latest[key] = payload
            c.wake.set()
