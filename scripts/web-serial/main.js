// G6 Arena Controller — Web Serial client.
//
// Talks to the Teensy 4.1 over its USB CDC port. The controller's
// SerialManager parses the same G4-compatible binary framing as the TCP
// path, so the wire format is identical to scripts/web/ and scripts/all_on.py:
//   binary command: [length, cmd, params...]
//   response:       [length, status, echo_cmd, ...ascii_message]
//
// Web Serial requires Chromium-based browser (Chrome / Edge / Opera) on a
// desktop OS. Firefox and Safari do not implement navigator.serial.

const $ = (id) => document.getElementById(id);
const log = (...parts) => {
  const line = parts.join(" ");
  const el = $("log");
  el.textContent += line + "\n";
  el.scrollTop = el.scrollHeight;
};
const hex = (bytes) =>
  Array.from(bytes).map((b) => b.toString(16).padStart(2, "0")).join(" ");
const printable = (bytes) =>
  Array.from(bytes)
    .map((b) => (b >= 0x20 && b < 0x7f ? String.fromCharCode(b) : "."))
    .join("");

let port = null;
let reader = null;
let writer = null;
let readLoopPromise = null;
// Accumulator for incoming bytes — used to parse [length, ...] responses out
// of the stream as they arrive.
let rxBuf = new Uint8Array(0);
// Resolvers for pending sendBytes() awaits, in order. Each holds the
// command echo byte it's waiting for plus a Promise resolver.
const pendingResponses = [];

if (!("serial" in navigator)) {
  $("api-check").hidden = false;
  $("btn-connect").disabled = true;
  log("Web Serial API not available in this browser.");
}

// Every command button is disabled until a port is open.
const COMMAND_BUTTONS = [
  "btn-all-on", "btn-all-off", "btn-stop", "btn-get-ip", "btn-get-info",
  "btn-set-refresh", "btn-trial", "btn-setframe", "btn-stream", "btn-upload",
  "btn-hex-send",
];

function setConnectedUI(connected) {
  $("btn-connect").disabled    = connected;
  $("btn-disconnect").disabled = !connected;
  for (const id of COMMAND_BUTTONS) $(id).disabled = !connected;
  const status = $("port-status");
  status.textContent = connected ? "connected" : "disconnected";
  status.classList.toggle("connected", connected);
}

// Little-endian uint16 → [lo, hi].
const u16le = (n) => [n & 0xff, (n >> 8) & 0xff];

// Append a chunk to rxBuf and pull complete [length, status, echo, ...]
// responses off the front. Resolves the next pendingResponses entry per
// frame extracted.
function consumeIncoming(chunk) {
  const merged = new Uint8Array(rxBuf.length + chunk.length);
  merged.set(rxBuf, 0);
  merged.set(chunk, rxBuf.length);
  rxBuf = merged;

  // The framing here is the same as NetworkManager / SerialManager:
  // first byte is the length of everything that follows. A response from
  // the controller looks like [N, status, echo_cmd, ...N-2 ascii bytes].
  while (rxBuf.length >= 1) {
    const claimedLen = rxBuf[0];
    const totalNeeded = 1 + claimedLen;
    if (rxBuf.length < totalNeeded) break;

    const frame = rxBuf.subarray(0, totalNeeded);
    const status   = frame[1];
    const echoCmd  = frame[2];
    const message  = frame.subarray(3);

    log(`           <- ${hex(frame)}`);
    log(`             status=${status} echo=0x${echoCmd.toString(16).padStart(2,"0")} ascii=${JSON.stringify(printable(message))}`);

    // Resolve the oldest pending sendBytes() await, regardless of echo
    // match — out-of-order responses are not expected on this transport
    // and would indicate a host/firmware desync worth surfacing in the
    // log rather than hiding.
    const pending = pendingResponses.shift();
    if (pending) pending.resolve({ status, echoCmd, message });

    rxBuf = rxBuf.subarray(totalNeeded);
  }
}

async function readLoop() {
  try {
    while (port && port.readable && reader) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value && value.length) consumeIncoming(value);
    }
  } catch (err) {
    if (err && err.name !== "AbortError") {
      log(`!! read error: ${err.message || err}`);
    }
  }
}

async function connect() {
  try {
    // Web Serial doesn't expose a vendor filter for the Teensy 4.1 USB CDC
    // by default — just let the user pick. The user-gesture chooser
    // remembers prior selections for the origin.
    port = await navigator.serial.requestPort();
    // baudRate is required by the API but ignored by USB CDC on the
    // device side (Teensy reports any rate as "OK"). 115200 is just a
    // conventional placeholder.
    await port.open({ baudRate: 115200 });

    writer = port.writable.getWriter();
    reader = port.readable.getReader();
    rxBuf = new Uint8Array(0);
    readLoopPromise = readLoop();

    setConnectedUI(true);
    log("-- connected to serial port");
  } catch (err) {
    log(`!! connect error: ${err && err.message ? err.message : err}`);
    port = null;
  }
}

async function disconnect() {
  setConnectedUI(false);
  // Reject any pending awaits so callers see the disconnect.
  for (const p of pendingResponses) {
    p.reject(new Error("port disconnected"));
  }
  pendingResponses.length = 0;
  try {
    if (reader) {
      await reader.cancel();
      try { reader.releaseLock(); } catch (_) {}
    }
  } catch (_) {}
  reader = null;
  try {
    if (writer) {
      try { writer.releaseLock(); } catch (_) {}
    }
  } catch (_) {}
  writer = null;
  try {
    if (readLoopPromise) await readLoopPromise;
  } catch (_) {}
  readLoopPromise = null;
  try {
    if (port) await port.close();
  } catch (_) {}
  port = null;
  log("-- disconnected");
}

// Send a command and return the decoded { status, echoCmd, message } response,
// or null on write error / timeout.
async function sendBytes(label, bytes) {
  if (!writer) {
    log("           !! not connected");
    return null;
  }
  // Stream frames are >1 kB — truncate the hex dump so the log stays readable.
  const shown = bytes.length > 32
    ? `${hex(Array.from(bytes).slice(0, 16))} … (${bytes.length} bytes)`
    : hex(bytes);
  log(`${label.padStart(10, " ")} -> ${shown}`);

  // Queue a response slot before writing so a fast reply can't lose its
  // promise.
  const respPromise = new Promise((resolve, reject) => {
    pendingResponses.push({ resolve, reject });
  });

  try {
    await writer.write(new Uint8Array(bytes));
  } catch (err) {
    log(`           !! write error: ${err.message || err}`);
    pendingResponses.pop();  // drop the slot we just queued
    return null;
  }

  // 500 ms soft timeout — controller responds in well under 1 ms.
  const timeout = new Promise((_, reject) =>
    setTimeout(() => reject(new Error("response timeout")), 500),
  );
  try {
    return await Promise.race([respPromise, timeout]);
  } catch (err) {
    log(`           !! ${err.message || err}`);
    // Remove this pending slot if still queued (timeout case).
    const idx = pendingResponses.findIndex((p) => p.resolve === respPromise);
    if (idx >= 0) pendingResponses.splice(idx, 1);
    return null;
  }
}

// get-controller-info (0x67) reply carries {version, capability_bitmap}.
function decodeControllerInfo(resp) {
  if (!resp || !resp.message || resp.message.length < 2) {
    log("             !! controller-info reply too short");
    return;
  }
  const version = resp.message[0];
  const cap = resp.message[1];
  const bits = [
    [0, "g6_mode"], [1, "v2_local_storage"], [2, "mode_1_tsi"],
    [3, "v3_triggered"], [4, "v3_gated"],
  ];
  const set = bits.filter(([b]) => cap & (1 << b)).map(([, n]) => n);
  log(`             controller version=${version} capability=0x${cap
        .toString(16).padStart(2, "0")} [${set.join(", ") || "none"}]`);
}

$("btn-connect").onclick    = connect;
$("btn-disconnect").onclick = disconnect;

$("btn-all-on").onclick  = () => sendBytes("all_on",  [0x01, 0xff]);
$("btn-all-off").onclick = () => sendBytes("all_off", [0x01, 0x00]);
$("btn-stop").onclick    = () => sendBytes("stop",    [0x01, 0x30]);
$("btn-get-ip").onclick  = () => sendBytes("get_ip",  [0x01, 0x66]);

$("btn-get-info").onclick = async () => {
  const resp = await sendBytes("get_info", [0x01, 0x67]);
  decodeControllerInfo(resp);
};

$("btn-set-refresh").onclick = () => {
  const hz = Math.max(1, Math.min(65535, parseInt($("refresh").value, 10) || 0));
  // [len=3, 0x16, lo, hi]
  sendBytes("set_refresh", [0x03, 0x16, ...u16le(hz)]);
};

$("btn-trial").onclick = () => {
  const mode = parseInt($("mode").value, 10);
  const pat  = Math.max(0, parseInt($("pat").value, 10)  || 0);
  const rate = Math.max(0, parseInt($("rate").value, 10) || 0);
  let   gain = parseInt($("gain").value, 10) || 0;
  if (gain < 0) gain += 256;            // int8 → byte
  const init = Math.max(0, parseInt($("init").value, 10) || 0);
  // trial-params (0x08): [len=0x0c, 0x08, mode, pat_lo, pat_hi, rate_lo,
  // rate_hi, gain, init_lo, init_hi, 0, 0, 0]  (3 reserved bytes pad to the
  // documented 12-byte combined-command length).
  const params = [
    mode & 0xff, ...u16le(pat), ...u16le(rate), gain & 0xff, ...u16le(init),
    0, 0, 0,
  ];
  sendBytes("trial", [0x0c, 0x08, ...params]);
};

$("btn-setframe").onclick = () => {
  const idx = Math.max(0, parseInt($("frame").value, 10) || 0);
  // set-frame-position (0x70): [len=3, 0x70, lo, hi]
  sendBytes("set_frame", [0x03, 0x70, ...u16le(idx)]);
};

// --- stream-frame (0x32, Mode 5) ------------------------------------------
// Builds a full G6 v1 frame of pre-formatted, parity-correct panel blocks
// client-side and streams it. This is the host's job for Mode 5 (the
// controller passes blocks through unchanged), so we replicate the panel-
// block format here: [header|parity][cmd][pixels][duty_cycle].
//   pixel layout: row-major, MSB-first, origin bottom-left (g6_04 § Pixel Data)
// The Stream dropdown offers the same preset patterns as the webDisplayTools
// G6 panel editor (ported below), prefixed GS2_ / GS16_.
const PANEL = 20;
const NUM_PANELS = 20;
const FRAME_PREFIX = 4;            // "FR" + index16
const GS2_BLOCK = 53, GS16_BLOCK = 203;

const popcount = (x) => { let c = 0; while (x) { c += x & 1; x >>>= 1; } return c; };

// Even parity over {version_bits, cmd, payload}; result goes in header bit 7.
function parityBit(versionByte, cmd, payload) {
  let ones = popcount(versionByte & 0x7f) + popcount(cmd);
  for (const b of payload) ones += popcount(b);
  return ones & 1;
}

// Pack a 20x20 intensity image into panel pixel bytes, matching the
// webDisplayTools / g6_04 convention: array row 0 is the BOTTOM row, origin
// bottom-left, pixel_num = row * 20 + col (no vertical flip). This keeps the
// arena output identical to the editor preview. GS2: 1 bit/pixel (non-zero =
// on). GS16: 4 bits/pixel, even pixel = high nibble.
function packImageBottomLeft(img, gs16) {
  const out = new Uint8Array(gs16 ? 200 : 50);
  for (let r = 0; r < PANEL; r++) {
    for (let c = 0; c < PANEL; c++) {
      const v = img[r][c] & (gs16 ? 0x0f : 0x01);
      if (!v) continue;
      const k = r * PANEL + c;                   // pixel_num 0..399
      if (gs16) {
        const bi = k >> 1;
        out[bi] = (k & 1) ? (out[bi] & 0xf0) | v : (out[bi] & 0x0f) | (v << 4);
      } else {
        out[k >> 3] |= 0x80 >> (k & 7);          // MSB-first
      }
    }
  }
  return out;
}

// Wrap raw panel pixel bytes (50 GS2 / 200 GS16) into a parity-correct G6 v1
// panel block: [header|parity][cmd][pixels][duty_cycle=0xff].
function wrapPixelBlock(gs16, pixels) {
  const blockLen = gs16 ? GS16_BLOCK : GS2_BLOCK;
  const cmd = gs16 ? 0x30 : 0x10;
  const block = new Uint8Array(blockLen);
  block[1] = cmd;
  block.set(pixels, 2);
  block[2 + pixels.length] = 0xff;        // duty_cycle = full brightness
  block[0] = 0x01 | (parityBit(0x01, cmd, block.subarray(2, blockLen)) << 7);
  return block;
}

// Concatenate a 4-byte "FR" + index0 prefix with 20 equal-length panel blocks.
function frameBodyFromBlocks(blocks) {
  const blockLen = blocks[0].length;
  const body = new Uint8Array(FRAME_PREFIX + NUM_PANELS * blockLen);
  body[0] = 0x46; body[1] = 0x52;          // "FR"; index16 stays 0
  for (let p = 0; p < NUM_PANELS; p++) {
    body.set(blocks[p], FRAME_PREFIX + p * blockLen);
  }
  return body;
}

// Wrap a frame body (prefix + blocks) in the 3-byte stream header.
function streamFraming(frameBody) {
  const out = new Uint8Array(3 + frameBody.length);
  out[0] = 0x32;
  out[1] = frameBody.length & 0xff;
  out[2] = (frameBody.length >> 8) & 0xff;
  out.set(frameBody, 3);
  return out;
}

// Stream the same 20x20 image to all 20 panels.
function streamImage(gs16, img) {
  const block = wrapPixelBlock(gs16, packImageBottomLeft(img, gs16));
  return streamFraming(frameBodyFromBlocks(Array(NUM_PANELS).fill(block)));
}

// Preset pattern generators, ported verbatim from the webDisplayTools G6 panel
// editor (reiserlab.github.io/webDisplayTools/g6_panel_editor.html). Each
// returns a fresh 20x20 array p[row][col]; row 0 = bottom, col 0 = left.
// GS2 values are 0/1; GS16 values are 0..15.
const GS2_PATTERNS = {
  all_off: () => Array(20).fill(null).map(() => Array(20).fill(0)),
  all_on: () => Array(20).fill(null).map(() => Array(20).fill(1)),
  border: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let i = 0; i < 20; i++) { p[0][i] = p[19][i] = p[i][0] = p[i][19] = 1; } return p; },
  cross: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let i = 0; i < 20; i++) { p[9][i] = p[10][i] = p[i][9] = p[i][10] = 1; } return p; },
  checker_2x2: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) p[r][c] = (Math.floor(r/2) + Math.floor(c/2)) % 2; return p; },
  horiz_stripes: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let row = 0; row < 20; row += 4) p[row].fill(1); return p; },
  vert_stripes: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let col = 0; col < 20; col += 4) for (let row = 0; row < 20; row++) p[row][col] = 1; return p; },
  arrow_up: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { p[10+i][9-i] = p[10+i][10+i] = 1; }
    for (let r = 0; r < 15; r++) { p[r][9] = p[r][10] = 1; }
    return p; },
  arrow_down: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { p[9-i][9-i] = p[9-i][10+i] = 1; }
    for (let r = 5; r < 20; r++) { p[r][9] = p[r][10] = 1; }
    return p; },
  arrow_left: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { p[9-i][9-i] = p[10+i][9-i] = 1; }
    for (let c = 0; c < 15; c++) { p[9][c] = p[10][c] = 1; }
    return p; },
  arrow_right: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { p[9-i][10+i] = p[10+i][10+i] = 1; }
    for (let c = 5; c < 20; c++) { p[9][c] = p[10][c] = 1; }
    return p; },
  circle: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, r=8;
    for (let row = 0; row < 20; row++) for (let col = 0; col < 20; col++) {
      const d = Math.sqrt((row-cy)**2 + (col-cx)**2);
      if (d >= r-0.7 && d <= r+0.7) p[row][col] = 1;
    } return p; },
  filled_circle: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, r=8;
    for (let row = 0; row < 20; row++) for (let col = 0; col < 20; col++) {
      if (Math.sqrt((row-cy)**2 + (col-cx)**2) <= r) p[row][col] = 1;
    } return p; },
  triangle_up: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let row = 2; row < 18; row++) {
      const halfWidth = Math.floor((row - 2) * 0.6);
      for (let c = 10 - halfWidth; c <= 9 + halfWidth; c++) {
        if (c >= 0 && c < 20) p[row][c] = 1;
      }
    } return p; },
  triangle_down: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let row = 2; row < 18; row++) {
      const halfWidth = Math.floor((17 - row) * 0.6);
      for (let c = 10 - halfWidth; c <= 9 + halfWidth; c++) {
        if (c >= 0 && c < 20) p[row][c] = 1;
      }
    } return p; },
  diamond: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, size=8;
    for (let row = 0; row < 20; row++) for (let col = 0; col < 20; col++) {
      const d = Math.abs(row-cy) + Math.abs(col-cx);
      if (d >= size-0.7 && d <= size+0.7) p[row][col] = 1;
    } return p; },
  filled_diamond: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, size=8;
    for (let row = 0; row < 20; row++) for (let col = 0; col < 20; col++) {
      if (Math.abs(row-cy) + Math.abs(col-cx) <= size) p[row][col] = 1;
    } return p; },
  star_5point: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, outerR=9, innerR=4;
    for (let i = 0; i < 5; i++) {
      const angle1 = (i * 72 - 90) * Math.PI / 180;
      const x1 = cx + outerR * Math.cos(angle1), y1 = cy + outerR * Math.sin(angle1);
      for (let t = 0; t <= 1; t += 0.02) {
        const px = Math.round(cx + t * (x1 - cx)), py = Math.round(cy + t * (y1 - cy));
        if (px >= 0 && px < 20 && py >= 0 && py < 20) p[py][px] = 1;
      }
    } return p; },
  left_half: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 20; r++) for (let c = 0; c < 10; c++) p[r][c] = 1; return p; },
  right_half: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 20; r++) for (let c = 10; c < 20; c++) p[r][c] = 1; return p; },
  top_half: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 10; r < 20; r++) for (let c = 0; c < 20; c++) p[r][c] = 1; return p; },
  bottom_half: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 10; r++) for (let c = 0; c < 20; c++) p[r][c] = 1; return p; },
  quadrant_tl: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 10; r < 20; r++) for (let c = 0; c < 10; c++) p[r][c] = 1; return p; },
  quadrant_tr: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 10; r < 20; r++) for (let c = 10; c < 20; c++) p[r][c] = 1; return p; },
  quadrant_bl: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 10; r++) for (let c = 0; c < 10; c++) p[r][c] = 1; return p; },
  quadrant_br: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 10; r++) for (let c = 10; c < 20; c++) p[r][c] = 1; return p; },
};

const GS16_PATTERNS = {
  gradient_h: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let row = 0; row < 20; row++) { const i = Math.floor(row / 19 * 15); for (let col = 0; col < 20; col++) p[row][col] = i; } return p; },
  gradient_v: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let col = 0; col < 20; col++) { const i = Math.floor(col / 19 * 15); for (let row = 0; row < 20; row++) p[row][col] = i; } return p; },
  gradient_diag: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0)); for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) p[r][c] = Math.min(15, Math.floor((r+c)/38*15)); return p; },
  concentric_bright: () => { const p=Array(20).fill(null).map(()=>Array(20).fill(0)); const cr=9.5,cc=9.5,md=Math.sqrt(cr*cr+cc*cc); for(let r=0;r<20;r++)for(let c=0;c<20;c++){const d=Math.sqrt((r-cr)**2+(c-cc)**2);p[r][c]=Math.max(0,Math.min(15,15-Math.floor(d/md*16)));}return p;},
  concentric_dark: () => { const p=Array(20).fill(null).map(()=>Array(20).fill(0)); const cr=9.5,cc=9.5,md=Math.sqrt(cr*cr+cc*cc); for(let r=0;r<20;r++)for(let c=0;c<20;c++){const d=Math.sqrt((r-cr)**2+(c-cc)**2);p[r][c]=Math.min(15,Math.floor(d/md*16));}return p;},
  radial_pulse: () => { const p=Array(20).fill(null).map(()=>Array(20).fill(0)); const cr=9.5,cc=9.5; for(let r=0;r<20;r++)for(let c=0;c<20;c++){const d=Math.sqrt((r-cr)**2+(c-cc)**2);const rd=Math.abs(d-2)+Math.abs(d-5)+Math.abs(d-8)+Math.abs(d-11);p[r][c]=Math.max(0,Math.min(15,15-Math.floor(rd/30*15)));}return p;},
  spotlight: () => { const p=Array(20).fill(null).map(()=>Array(20).fill(0)); const cr=9.5,cc=9.5,md=Math.sqrt(cr*cr+cc*cc); for(let r=0;r<20;r++)for(let c=0;c<20;c++){const d=Math.sqrt((r-cr)**2+(c-cc)**2);p[r][c]=Math.max(0,Math.min(15,Math.floor(15*Math.exp(-3*d/md))));}return p;},
  starfield: () => { const p=Array(20).fill(null).map(()=>Array(20).fill(0)); [[2,3,15],[5,8,12],[1,15,14],[8,2,10],[12,18,15],[16,7,13],[4,12,11],[18,16,15],[7,5,9],[14,14,14],[3,18,8],[19,4,12],[10,10,15],[15,2,11],[6,19,10],[11,6,13],[17,12,14],[9,17,9],[13,9,12],[1,7,7]].forEach(([r,c,i])=>{p[r][c]=i;[[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]].forEach(([dr,dc])=>{const nr=r+dr,nc=c+dc;if(nr>=0&&nr<20&&nc>=0&&nc<20)p[nr][nc]=Math.max(p[nr][nc],Math.floor(i*0.4));});});return p;},
  circles_varied: () => { const p=Array(20).fill(null).map(()=>Array(20).fill(0)); [[5,5,3,15],[5,15,2,10],[15,5,4,12],[15,15,2.5,8]].forEach(([cr,cc,rad,i])=>{for(let r=0;r<20;r++)for(let c=0;c<20;c++){if(Math.sqrt((r-cr)**2+(c-cc)**2)<=rad)p[r][c]=Math.max(p[r][c],i);}});return p;},
  checkerboard_intensity: () => { const p=Array(20).fill(null).map(()=>Array(20).fill(0)); for(let r=0;r<20;r++)for(let c=0;c<20;c++){p[r][c]=(Math.floor(r/4)+Math.floor(c/4))%2===0?15:5;}return p;},
  arrow_up_gradient: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { const v = 15 - i; p[10+i][9-i] = p[10+i][10+i] = v; }
    for (let r = 0; r < 15; r++) { const v = Math.floor(15 * r / 14); p[r][9] = p[r][10] = v; }
    return p; },
  arrow_down_gradient: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { const v = 15 - i; p[9-i][9-i] = p[9-i][10+i] = v; }
    for (let r = 5; r < 20; r++) { const v = Math.floor(15 * (19 - r) / 14); p[r][9] = p[r][10] = v; }
    return p; },
  arrow_left_gradient: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { const v = 15 - i; p[9-i][9-i] = p[10+i][9-i] = v; }
    for (let c = 0; c < 15; c++) { const v = Math.floor(15 * c / 14); p[9][c] = p[10][c] = v; }
    return p; },
  arrow_right_gradient: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let i = 0; i < 10; i++) { const v = 15 - i; p[9-i][10+i] = p[10+i][10+i] = v; }
    for (let c = 5; c < 20; c++) { const v = Math.floor(15 * (19 - c) / 14); p[9][c] = p[10][c] = v; }
    return p; },
  expanding_disc_small: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, r=3;
    for (let row = 0; row < 20; row++) for (let col = 0; col < 20; col++) {
      const d = Math.sqrt((row-cy)**2 + (col-cx)**2);
      if (d <= r) p[row][col] = 15; else if (d <= r+2) p[row][col] = Math.max(0, 15 - Math.floor((d-r)*5));
    } return p; },
  expanding_disc_medium: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, r=6;
    for (let row = 0; row < 20; row++) for (let col = 0; col < 20; col++) {
      const d = Math.sqrt((row-cy)**2 + (col-cx)**2);
      if (d <= r) p[row][col] = 15; else if (d <= r+2) p[row][col] = Math.max(0, 15 - Math.floor((d-r)*5));
    } return p; },
  expanding_disc_large: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, r=9;
    for (let row = 0; row < 20; row++) for (let col = 0; col < 20; col++) {
      const d = Math.sqrt((row-cy)**2 + (col-cx)**2);
      if (d <= r) p[row][col] = 15; else if (d <= r+2) p[row][col] = Math.max(0, 15 - Math.floor((d-r)*5));
    } return p; },
  sine_horiz_1: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) p[r][c] = Math.round(7.5 + 7.5 * Math.sin(2 * Math.PI * c / 20));
    return p; },
  sine_horiz_2: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) p[r][c] = Math.round(7.5 + 7.5 * Math.sin(2 * Math.PI * c / 10));
    return p; },
  sine_vert_1: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) p[r][c] = Math.round(7.5 + 7.5 * Math.sin(2 * Math.PI * r / 20));
    return p; },
  sine_vert_2: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) p[r][c] = Math.round(7.5 + 7.5 * Math.sin(2 * Math.PI * r / 10));
    return p; },
  gaussian_center: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=9.5, cy=9.5, sigma=4;
    for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) { const d2 = (r-cy)**2 + (c-cx)**2; p[r][c] = Math.round(15 * Math.exp(-d2 / (2 * sigma * sigma))); }
    return p; },
  gaussian_offset: () => { const p = Array(20).fill(null).map(() => Array(20).fill(0));
    const cx=14, cy=14, sigma=3;
    for (let r = 0; r < 20; r++) for (let c = 0; c < 20; c++) { const d2 = (r-cy)**2 + (c-cx)**2; p[r][c] = Math.round(15 * Math.exp(-d2 / (2 * sigma * sigma))); }
    return p; },
};

// Human-readable label from a snake_case key (e.g. "arrow_up" -> "Arrow Up",
// "quadrant_tl" -> "Quadrant TL", "checker_2x2" -> "Checker 2×2").
function titleizePattern(key) {
  return key.split("_").map((t) => {
    if (t === "2x2") return "2×2";
    if (/^(tl|tr|bl|br|h|v)$/.test(t)) return t.toUpperCase();
    return t.charAt(0).toUpperCase() + t.slice(1);
  }).join(" ");
}

// Populate the Stream dropdown with the editor's GS2_/GS16_ presets.
(function populateStreamOptions() {
  const sel = $("stream");
  sel.innerHTML = "";
  const addGroup = (prefix, table) => {
    for (const key of Object.keys(table)) {
      const o = document.createElement("option");
      o.value = `${prefix}_${key}`;
      o.textContent = `${prefix}_${titleizePattern(key)}`;
      sel.appendChild(o);
    }
  };
  addGroup("GS2", GS2_PATTERNS);
  addGroup("GS16", GS16_PATTERNS);
})();

$("btn-stream").onclick = () => {
  const sel = $("stream").value;                 // e.g. "GS2_all_on"
  const gs16 = sel.startsWith("GS16_");
  const key = sel.slice(gs16 ? 5 : 4);
  const gen = (gs16 ? GS16_PATTERNS : GS2_PATTERNS)[key];
  if (!gen) { log(`           !! unknown pattern ${sel}`); return; }
  sendBytes(`stream:${sel}`, streamImage(gs16, gen()));
};

// --- .bin file upload -> stream (0x32) ------------------------------------
// Accepts the .bin formats webDisplayTools emits and assembles a full arena
// stream frame. Classified by size (and by the G6PT magic for .pat files):
//
//   50  / 200          one panel's raw pixels (GS2 / GS16)  -> replicate x20
//   1000 / 4000        20 panels' raw pixels (GS2 / GS16)   -> one per panel
//   53  / 203          one pre-formatted panel block        -> replicate x20
//   1060 / 4060        20 pre-formatted blocks (no prefix)  -> prepend prefix
//   1064 / 4064        a full frame (prefix + blocks)       -> stream as-is
//   "G6PT…"            a v2 .pat container                  -> stream frame 0
//
// Raw pixels are wrapped into panel blocks here (header + cmd + duty_cycle +
// parity); pre-formatted bytes are passed through untouched (the host owns
// their parity, exactly as in Mode 5).
const GS2_PIXELS = 50, GS16_PIXELS = 200;

function frameBodyFromUpload(buf) {
  const n = buf.length;

  // Raw per-panel pixels: replicate one panel across the arena.
  const replicatePixels = (gs16) => {
    const blk = wrapPixelBlock(gs16, buf);
    return { gs16, body: frameBodyFromBlocks(Array(NUM_PANELS).fill(blk)) };
  };
  // Raw full-arena pixels: one panel's worth per block, row-major.
  const perPanelPixels = (gs16, plen) => {
    const blocks = [];
    for (let p = 0; p < NUM_PANELS; p++) {
      blocks.push(wrapPixelBlock(gs16, buf.subarray(p * plen, (p + 1) * plen)));
    }
    return { gs16, body: frameBodyFromBlocks(blocks) };
  };
  // Pre-formatted single block: replicate verbatim (host already set parity).
  const replicateBlock = (gs16) =>
    ({ gs16, body: frameBodyFromBlocks(Array(NUM_PANELS).fill(buf)) });
  // Pre-formatted blocks with no FR prefix: prepend one.
  const prefixBlocks = (gs16) => {
    const body = new Uint8Array(FRAME_PREFIX + n);
    body[0] = 0x46; body[1] = 0x52;
    body.set(buf, FRAME_PREFIX);
    return { gs16, body };
  };

  switch (n) {
    case GS2_PIXELS:          return replicatePixels(false);
    case GS16_PIXELS:         return replicatePixels(true);
    case GS2_PIXELS  * NUM_PANELS: return perPanelPixels(false, GS2_PIXELS);   // 1000
    case GS16_PIXELS * NUM_PANELS: return perPanelPixels(true,  GS16_PIXELS);  // 4000
    case GS2_BLOCK:           return replicateBlock(false);
    case GS16_BLOCK:          return replicateBlock(true);
    case GS2_BLOCK  * NUM_PANELS:  return prefixBlocks(false);                 // 1060
    case GS16_BLOCK * NUM_PANELS:  return prefixBlocks(true);                  // 4060
    case FRAME_PREFIX + GS2_BLOCK  * NUM_PANELS: return { gs16: false, body: buf }; // 1064
    case FRAME_PREFIX + GS16_BLOCK * NUM_PANELS: return { gs16: true,  body: buf }; // 4064
    default: return null;
  }
}

// v2 .pat container: take frame 0's body (prefix + blocks, minus the CRC-16
// trailer) straight from the file — it is already a valid stream payload.
function patFrameBody(buf) {
  const gs16 = buf[10] === 2;          // gs_val: 1=GS2, 2=GS16
  const rows = buf[8], cols = buf[9];
  const blockLen = gs16 ? GS16_BLOCK : GS2_BLOCK;
  const bodyLen = FRAME_PREFIX + rows * cols * blockLen;  // excl. CRC-16
  return { gs16, rows, cols, body: buf.subarray(18, 18 + bodyLen) };
}

async function streamBinFile(file) {
  let buf;
  try {
    buf = new Uint8Array(await file.arrayBuffer());
  } catch (err) {
    log(`           !! read error: ${err.message || err}`);
    return;
  }
  const n = buf.length;

  let res;
  if (n >= 18 && buf[0] === 0x47 && buf[1] === 0x36 &&
      buf[2] === 0x50 && buf[3] === 0x54) {            // "G6PT"
    res = patFrameBody(buf);
    log(`           ${file.name}: G6PT .pat (${n} B, ${res.rows}x${res.cols} ` +
        `${res.gs16 ? "GS16" : "GS2"}) — streaming frame 0`);
  } else {
    res = frameBodyFromUpload(buf);
  }

  if (!res) {
    log(`           !! ${file.name}: unrecognized size ${n} B.`);
    log("              expected per-panel pixels (50/200), full-arena pixels");
    log("              (1000/4000), blocks (53/203/1060/4060), a frame (1064/4064),");
    log("              or a G6PT .pat file.");
    return;
  }
  log(`           ${file.name}: ${n} B -> ${res.gs16 ? "GS16" : "GS2"} stream ` +
      `frame (${res.body.length}-byte payload)`);
  await sendBytes(`bin:${file.name}`, streamFraming(res.body));
}

$("btn-upload").onclick = () => {
  const f = $("binfile").files && $("binfile").files[0];
  if (!f) {
    log("           !! choose a .bin file first");
    return;
  }
  streamBinFile(f);
};

$("btn-hex-send").onclick = () => {
  const raw = $("hex").value.trim().replace(/0x/gi, "").replace(/[^0-9a-fA-F]/g, "");
  if (raw.length === 0) {
    log("           !! hex input is empty");
    return;
  }
  if (raw.length % 2 !== 0) {
    log("           !! hex input has odd character count");
    return;
  }
  const bytes = [];
  for (let i = 0; i < raw.length; i += 2) {
    bytes.push(parseInt(raw.slice(i, i + 2), 16));
  }
  sendBytes("hex", bytes);
};
