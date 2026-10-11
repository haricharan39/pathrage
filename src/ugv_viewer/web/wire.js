// Binary frame parsing, kept free of DOM/three.js so it can be tested with node.
// Layout is defined in ugv_viewer/viewer_core.py (all arrays 4-byte aligned).
export const MSG_MAP_DELTA = 1, MSG_LIVE = 2, MSG_TIER1 = 3;

const pad4 = (n) => (4 - (n % 4)) % 4;

export function msgType(buf) { return new DataView(buf).getUint8(0); }
export function msgCount(buf) { return new DataView(buf).getUint32(4, true); }

export function parseMapDelta(buf) {
  const n = msgCount(buf);
  const rgbOff = 8 + 16 * n;
  return {
    n,
    idx: new Uint32Array(buf, 8, n),
    pos: new Float32Array(buf, 8 + 4 * n, 3 * n),
    rgb: new Uint8Array(buf, rgbOff, 3 * n),
    cls: new Uint8Array(buf, rgbOff + 3 * n + pad4(3 * n), n),
  };
}

export function parseLive(buf) {
  const n = msgCount(buf);
  const rgbOff = 8 + 12 * n;
  return {
    n,
    pos: new Float32Array(buf, 8, 3 * n),
    rgb: new Uint8Array(buf, rgbOff, 3 * n),
    cls: new Uint8Array(buf, rgbOff + 3 * n + pad4(3 * n), n),
  };
}

export function parseTier1(buf) {
  const n = msgCount(buf);
  return { n, pos: new Float32Array(buf, 8, 3 * n) };
}
