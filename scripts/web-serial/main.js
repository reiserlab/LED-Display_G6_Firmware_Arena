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
  "btn-set-refresh", "btn-trial", "btn-setframe", "btn-stream", "btn-hex-send",
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

// Pack a 20x20 intensity image (img[row][col], row 0 = top) into panel pixel
// bytes. GS2: 1 bit/pixel (any non-zero = on). GS16: 4 bits/pixel.
function packPixels(img, gs16) {
  const out = new Uint8Array(gs16 ? 200 : 50);
  for (let r = 0; r < PANEL; r++) {
    const rowFromBottom = PANEL - 1 - r;
    for (let c = 0; c < PANEL; c++) {
      const v = img[r][c] & (gs16 ? 0x0f : 0x01);
      if (!v) continue;
      const k = rowFromBottom * PANEL + c;       // pixel_num 0..399
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

// Assemble [0x32, len_lo, len_hi, "FR", idx16, block0..19] for the chosen
// test pattern. imgFn(panelIndex) returns a fresh 20x20 image.
function buildStreamFrame(gs16, imgFn) {
  const blockLen = gs16 ? GS16_BLOCK : GS2_BLOCK;
  const cmd = gs16 ? 0x30 : 0x10;
  const frameLen = FRAME_PREFIX + NUM_PANELS * blockLen;
  const frame = new Uint8Array(frameLen);
  frame[0] = 0x46; frame[1] = 0x52;  // "FR"; index16 stays 0

  for (let p = 0; p < NUM_PANELS; p++) {
    const pixels = packPixels(imgFn(p), gs16);
    const block = frame.subarray(FRAME_PREFIX + p * blockLen);
    block[0] = 0x01;            // version v1 (parity set below)
    block[1] = cmd;
    block.set(pixels, 2);
    block[2 + pixels.length] = 0xff;  // duty_cycle = full brightness
    const payload = block.subarray(2, blockLen);
    block[0] = 0x01 | (parityBit(0x01, cmd, payload) << 7);
  }

  // Stream framing: 3-byte header [0x32, len_lo, len_hi] then frame bytes.
  const out = new Uint8Array(3 + frameLen);
  out[0] = 0x32; out[1] = frameLen & 0xff; out[2] = (frameLen >> 8) & 0xff;
  out.set(frame, 3);
  return out;
}

const solidImage = (v) =>
  Array.from({ length: PANEL }, () => new Array(PANEL).fill(v));
const checkerImage = () =>
  Array.from({ length: PANEL }, (_, r) =>
    Array.from({ length: PANEL }, (_, c) => (r + c) & 1));

$("btn-stream").onclick = () => {
  const mode = $("stream").value;
  let gs16 = false;
  let imgFn;
  if (mode === "gs16-on")       { gs16 = true;  imgFn = () => solidImage(0x0f); }
  else if (mode === "gs2-checker") {            imgFn = () => checkerImage(); }
  else                          {               imgFn = () => solidImage(1); }
  sendBytes(`stream:${mode}`, buildStreamFrame(gs16, imgFn));
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
