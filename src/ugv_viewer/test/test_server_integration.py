"""Starts the real aiohttp server and talks to it like the browser does (no ROS needed)."""
import asyncio
import json
import pathlib
import socket
import subprocess
import tempfile

import aiohttp
import numpy as np

from ugv_viewer import viewer_core as vc
from ugv_viewer.server import ViewerServer

WEB = pathlib.Path(__file__).resolve().parents[1] / "web"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def make_map(n=300):
    m = vc.VoxelMap(voxel_size=0.1)
    pts = (np.random.rand(n, 3) * 4).astype(np.float32)
    m.integrate(pts, None, np.zeros(n, np.uint8))
    return m


def test_snapshot_live_commands_and_static():
    m = make_map()
    cmds = []
    port = free_port()

    def snapshot():
        return [json.dumps({"type": "hello", "max_points": 1000, "voxel": 0.1, "fixed_frame": "odom"}),
                *m.snapshot_chunks(chunk=100), json.dumps({"type": "traj_set", "pts": [[0, 0, 0]]})]

    srv = ViewerServer(WEB, "127.0.0.1", port, snapshot, cmds.append)
    srv.start()

    async def run():
        async with aiohttp.ClientSession() as sess:
            async with sess.get(f"http://127.0.0.1:{port}/") as r:
                assert r.status == 200 and "VisionNav" in await r.text()
            for path in ("app.js", "wire.js", "vendor/three.module.min.js", "vendor/OrbitControls.js"):
                async with sess.get(f"http://127.0.0.1:{port}/static/{path}") as r:
                    assert r.status == 200, path

            async with sess.ws_connect(f"http://127.0.0.1:{port}/ws") as ws:
                got_pts, hello, traj = 0, False, False
                for _ in range(1 + 3 + 1):
                    msg = await asyncio.wait_for(ws.receive(), 3)
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        d = json.loads(msg.data)
                        hello |= d["type"] == "hello"
                        traj |= d["type"] == "traj_set"
                    else:
                        got_pts += vc.unpack(msg.data)[1].size
                assert hello and traj and got_pts == len(m)

                # a live frame and a tier-1 frame are delivered (latest-wins channel)
                pos = np.random.rand(50, 3).astype(np.float32)
                srv.broadcast_latest("live", vc.pack_live(pos, np.zeros((50, 3), np.uint8),
                                                          np.ones(50, np.uint8)))
                srv.broadcast_latest("tier1", vc.pack_tier1(pos[:10]))
                kinds = set()
                for _ in range(2):
                    msg = await asyncio.wait_for(ws.receive(), 3)
                    kinds.add(vc.unpack(msg.data)[0])
                assert kinds == {vc.MSG_LIVE, vc.MSG_TIER1}

                # reliable delta arrives in order
                srv.broadcast_reliable(json.dumps({"type": "traj_add", "pts": [[1, 1, 0]]}))
                msg = await asyncio.wait_for(ws.receive(), 3)
                assert json.loads(msg.data)["type"] == "traj_add"

                # commands reach the handler; malformed ones do not kill the socket
                await ws.send_str(json.dumps({"cmd": "clear_map"}))
                await ws.send_str("not json")
                await ws.send_str(json.dumps({"cmd": "goal", "x": 1.5, "y": -2.0}))
                await asyncio.sleep(0.3)
                assert not ws.closed
            await asyncio.sleep(0.2)

    asyncio.run(run())
    assert [c["cmd"] for c in cmds] == ["clear_map", "goal"]
    srv.stop()


def test_late_joiner_gets_full_map():
    m = make_map(500)
    port = free_port()
    srv = ViewerServer(WEB, "127.0.0.1", port, lambda: list(m.snapshot_chunks(chunk=200)), None)
    srv.start()

    async def run():
        async with aiohttp.ClientSession() as sess:
            for _ in range(2):  # two separate clients each get everything
                async with sess.ws_connect(f"http://127.0.0.1:{port}/ws") as ws:
                    total = 0
                    while total < len(m):
                        msg = await asyncio.wait_for(ws.receive(), 3)
                        total += vc.unpack(msg.data)[1].size
                    assert total == len(m)

    asyncio.run(run())
    srv.stop()


def test_port_in_use_is_reported():
    port = free_port()
    a = ViewerServer(WEB, "127.0.0.1", port)
    a.start()
    b = ViewerServer(WEB, "127.0.0.1", port)
    try:
        b.start()
        raised = False
    except RuntimeError as exc:
        raised = "failed to start" in str(exc)
    assert raised
    a.stop()


def test_js_parser_matches_python_packing():
    """Pack frames in Python, parse them with the browser's wire.js under node."""
    n = 7
    idx = np.arange(n, dtype=np.uint32) + 10
    pos = np.random.rand(n, 3).astype(np.float32)
    rgb = (np.random.rand(n, 3) * 255).astype(np.uint8)
    cls = (np.arange(n) % 3).astype(np.uint8)
    d = pathlib.Path(tempfile.mkdtemp())
    (d / "delta.bin").write_bytes(vc.pack_map_delta(idx, pos, rgb, cls))
    (d / "live.bin").write_bytes(vc.pack_live(pos, rgb, cls))
    (d / "t1.bin").write_bytes(vc.pack_tier1(pos))
    (d / "expect.json").write_text(json.dumps({
        "idx": idx.tolist(), "pos": pos.ravel().tolist(), "rgb": rgb.ravel().tolist(),
        "cls": cls.tolist()}))
    script = f"""
import {{ readFileSync }} from 'node:fs';
import * as w from '{(WEB / "wire.js").as_posix()}';
const ab = (f) => {{ const b = readFileSync('{d.as_posix()}/' + f); return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength); }};
const e = JSON.parse(readFileSync('{d.as_posix()}/expect.json'));
const eq = (a, b, name) => {{ if (a.length !== b.length || a.some((v, i) => Math.abs(v - b[i]) > 1e-6)) {{ console.error('MISMATCH', name); process.exit(1); }} }};
let f = w.parseMapDelta(ab('delta.bin'));
eq([...f.idx], e.idx, 'idx'); eq([...f.pos], e.pos, 'pos'); eq([...f.rgb], e.rgb, 'rgb'); eq([...f.cls], e.cls, 'cls');
f = w.parseLive(ab('live.bin'));
eq([...f.pos], e.pos, 'live pos'); eq([...f.rgb], e.rgb, 'live rgb'); eq([...f.cls], e.cls, 'live cls');
f = w.parseTier1(ab('t1.bin'));
eq([...f.pos], e.pos, 't1 pos');
console.log('js-ok');
"""
    (d / "t.mjs").write_text(script)
    out = subprocess.run(["node", str(d / "t.mjs")], capture_output=True, text=True)
    assert out.returncode == 0 and "js-ok" in out.stdout, out.stderr


def test_demo_runs_and_streams():
    """The ROS-free demo should start, stream a growing map and accept commands."""
    import threading
    from ugv_viewer import demo

    port = free_port()
    t = threading.Thread(target=demo.main, args=(["--port", str(port), "--hz", "20"],), daemon=True)
    t.start()

    async def run():
        async with aiohttp.ClientSession() as sess:
            for _ in range(50):                      # wait for the server
                try:
                    async with sess.get(f"http://127.0.0.1:{port}/") as r:
                        break
                except aiohttp.ClientConnectionError:
                    await asyncio.sleep(0.1)
            async with sess.ws_connect(f"http://127.0.0.1:{port}/ws") as ws:
                voxels, pose_seen, live_seen = 0, False, False
                end = asyncio.get_event_loop().time() + 4
                while asyncio.get_event_loop().time() < end:
                    msg = await asyncio.wait_for(ws.receive(), 3)
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        d = json.loads(msg.data)
                        pose_seen |= d["type"] == "state" and "pose" in d
                    else:
                        u = vc.unpack(msg.data)
                        if u[0] == vc.MSG_MAP_DELTA:
                            voxels += u[1].size
                        live_seen |= u[0] == vc.MSG_LIVE
                assert voxels > 1000 and pose_seen and live_seen

    asyncio.run(run())
