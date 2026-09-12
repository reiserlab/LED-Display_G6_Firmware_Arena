#include <Arduino.h>
#include "NetworkManager.h"
#include "SerialManager.h"
#include "SpiManager.h"
#include "SdManager.h"
#include "CommandProcessor.h"
#include "Health.h"
#include "Version.h"
#include "Telemetry.h"

NetworkManager   net;
SerialManager    serial;
SpiManager       spi;
SdManager        sd;
CommandProcessor cmdProc(net, serial, spi, sd);

// DEBUG_SERIAL diagnostics default OFF so web serial clients get a clean
// command/response channel on fresh connect without needing to send a mute
// command first. CIPO capture scripts enable explicitly via SET_DIAG_OUTPUT
// (0xC3). State persists across USB reconnects but resets on power cycle.
volatile bool g_dbg_on = false;

#ifdef DEBUG_SERIAL
static bool ipPrinted = false;

// Sentinel-prefixes every line so this shares the same demux convention as
// DBG_PRINTF (constants.h) instead of injecting bare ASCII into the USB-CDC
// binary-response channel. Can't gate on g_dbg_on like DBG_PRINTF does --
// a host can only set that AFTER boot, and this needs to fire during boot
// itself -- so unlike DBG_PRINTF it prints unconditionally whenever
// DEBUG_SERIAL is compiled in. It still never blocks the boot sequence:
// no Serial.flush(), and each byte checks availableForWrite() first and
// silently drops instead of waiting on a full/absent host.
class SentinelPrint : public Print {
 public:
  size_t write(uint8_t c) override {
    if (Serial.availableForWrite() < (at_line_start_ ? 2 : 1)) return 0;
    if (at_line_start_) {
      Serial.write(AC::constants::diag_line_sentinel);
      at_line_start_ = false;
    }
    Serial.write(c);
    if (c == '\n') at_line_start_ = true;
    return 1;
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    size_t n = 0;
    for (size_t i = 0; i < size; ++i) n += write(buffer[i]);
    return n;
  }

 private:
  bool at_line_start_ = true;
};
#endif

void blinkStartupPattern();
void setupInterruptPriorities();

void setup() {
  // FIRST: capture + clear the reset cause and harvest the previous boot's
  // breadcrumb before anything else runs (GET_HEALTH 0xCA, issue #50).
  Health::begin();
  // Telemetry ring (0xA8/0xA9): keep a valid OCRAM ring from the previous boot
  // (the crash dump) or initialise it, then append STATE(boot). After
  // Health::begin() — the boot record carries the reset cause + breadcrumb.
  Telemetry::begin();

#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  delay(50);  // let CDC settle so this print isn't lost before the host attaches
  // TEMPORARY crash-loop diagnostic -- remove once root-caused.
  {
    SentinelPrint diag;
    // Build identity — same values GET_FIRMWARE_VERSION (0xCB) reports (src/Version.h).
    diag.printf("=== G6 arena controller fw %s%s (%s) built %s, %ux%u panels%s ===\n",
                AC::version::fw_git_sha, AC::version::fw_git_dirty ? "-dirty" : "",
                AC::version::fw_git_branch, AC::version::fw_build_date,
                (unsigned)AC::constants::panel_count_per_frame_row,
                (unsigned)AC::constants::panel_count_per_frame_col,
                AC::version::fw_debug_build ? " DEBUG_SERIAL" : "");
    uint32_t srsr = Health::stats.reset_cause;  // SRC_SRSR itself is cleared by Health::begin()
    if (Health::stats.prev_isr_valid && Health::stats.prev_wdog_fired) {
      diag.printf("=== WATCHDOG reset: previous boot hung at pc=0x%08lX lr=0x%08lX (isr entries %lu) ===\n",
                  (unsigned long)Health::stats.prev_wdog_pc, (unsigned long)Health::stats.prev_wdog_lr,
                  (unsigned long)Health::stats.prev_isr_count);
    }
    if (Health::stats.prev_valid) {
      diag.printf("=== health breadcrumb from previous boot: op=%u arg=0x%02X at %lu us; slowest op=%u %lu us ===\n",
                  (unsigned)Health::stats.prev_last_op, (unsigned)Health::stats.prev_op_arg,
                  (unsigned long)Health::stats.prev_stamp_us,
                  (unsigned)Health::stats.prev_slow_op, (unsigned long)Health::stats.prev_slow_us);
    }
    diag.printf("=== SRC_SRSR (reset cause) = 0x%08lX ===\n", (unsigned long)srsr);
    if (srsr & SRC_SRSR_IPP_USER_RESET_B)     diag.println("  IPP_USER_RESET_B (reset pin / button)");
    if (srsr & SRC_SRSR_CSU_RESET_B)          diag.println("  CSU_RESET_B");
    if (srsr & SRC_SRSR_WDOG_RST_B)           diag.println("  WDOG_RST_B (watchdog 1 timeout)");
    if (srsr & SRC_SRSR_WDOG3_RST_B)          diag.println("  WDOG3_RST_B (watchdog 3 timeout)");
    if (srsr & SRC_SRSR_LOCKUP_SYSRESETREQ)   diag.println("  LOCKUP_SYSRESETREQ (CPU lockup or software SCB_AIRCR reset)");
    if (srsr & SRC_SRSR_JTAG_RST_B)           diag.println("  JTAG_RST_B");
    if (srsr & SRC_SRSR_JTAG_SW_RST)          diag.println("  JTAG_SW_RST");
    if (srsr & SRC_SRSR_IPP_RESET_B)          diag.println("  IPP_RESET_B (power-on reset)");
    if (srsr & SRC_SRSR_TEMPSENSE_RST_B)      diag.println("  TEMPSENSE_RST_B");
    diag.printf("=== telemetry ring: %s, boot_count=%lu next_seq=%lu pending=%lu B dropped=%lu ===\n",
                Telemetry::keptAcrossReset() ? "KEPT across reset" : "initialised",
                (unsigned long)Telemetry::bootCount(), (unsigned long)Telemetry::nextSeq(),
                (unsigned long)Telemetry::pendingBytes(), (unsigned long)Telemetry::dropped());
    if (CrashReport) {
      diag.println("=== CrashReport (previous reset) ===");
      diag.print(CrashReport);
      diag.println("=====================================");
    } else {
      diag.println("=== no CrashReport (not a CPU fault) ===");
    }
  }
#endif

  // LED_BUILTIN is shared with SCK on SPI bus B0 (D13). Drive the boot blink
  // BEFORE SPI.begin() takes over the pin — never digitalWrite(LED_BUILTIN, ...)
  // after the SPI bus is up, or the SCK will glitch during traffic.
  blinkStartupPattern();

  // DEBUG BISECT: NetworkManager::begin() (QNEthernet) hangs the whole board
  // (including USB) on this unit, which has no Ethernet link. Disabled so
  // the bench build stays usable over USB/SPI. See debug write-up before
  // restoring — root cause not yet fixed.
  // net.begin();
  serial.begin();
  cmdProc.begin();
  spi.begin();
  sd.begin();  // mounts BUILTIN_SDCARD for Modes 2/3/4; safe with no card

  setupInterruptPriorities();

  // LAST: arm the 2 s hardware watchdog (RTWDOG, Health.h). Everything above
  // (SD mount, boot blink, USB settle) ran unguarded; from here loop() must
  // kick it every iteration or the controller resets WITH the breadcrumb +
  // telemetry ring intact (the #50 hang becomes a self-healing reboot).
  Health::watchdogBegin();
}

// The external-trigger input path (BNC "Digital IO 2 (5V)"/J4 -> U3 SN74LVC1T45
// -> J30 shunt -> R216 -> TNY.EINT fanout -> all panels' EINT/GP45) is now set
// up by CommandProcessor::begin() as DIO role `in_trigger` on port 2
// (SET_DIO_ROLE 0xAC, #135) — the old setupExternalTriggerInput() here flipped
// U3.DIR after begin() had made D35 an output, leaving D35 driving the EINT
// net against U3 (boot contention). applyDioRole tri-states D35 first.

void loop() {
  Health::loopTick();         // 0.  loop-iteration timing + count (GET_HEALTH 0xCA, #50)
  // Must run BEFORE net.serviceTcp(): net_'s client_ is a single reused slot,
  // so if the client owning an active 0x84/0x85/0x8A transfer disconnected,
  // serviceTcp() below would silently swap in a brand-new client on the same
  // slot and start parsing/streaming its bytes before this ever got a chance
  // to see the OLD (dead) connection state (PR #27 review point 5).
  cmdProc.serviceDisconnects();
  // net.serviceTcp();           // 1a. Accept TCP client, read and parse commands
  serial.serviceUsb();        // 1b. Read and parse commands from USB CDC
  cmdProc.processCommand();   // 2.  Handle one parsed command per source
  cmdProc.serviceDisplay();   // 3.  Re-transmit current frame at refresh rate
  Telemetry::service();       // 3a. Telemetry ring: heap guard + synthetic producer (T1); one compare when idle
  cmdProc.serviceDownload();  // 3b. Stream one 0x84 download chunk, if one is in flight
  cmdProc.serviceUpload();    // 3c. Stream one 0x85 upload chunk, if one is in flight
  cmdProc.serviceArchive();   // 3d. Stream one 0x8A archive step, if one is in flight
  // net.flushResponses();       // 4a. Send queued responses over TCP
  serial.flushResponses();    // 4b. Send queued responses over USB CDC

#ifdef DEBUG_SERIAL
  if (!ipPrinted && Serial && net.ipAddress()[0] != '\0') {
    DBG_PRINTF("MAC: %s  IP: %s\n", net.macAddress(), net.ipAddress());
    ipPrinted = true;
  }
#endif
}

void blinkStartupPattern() {
  static constexpr uint8_t  LED_PIN  = LED_BUILTIN;
  static constexpr uint32_t SHORT_MS = 100;
  static constexpr uint32_t LONG_MS  = 300;
  static constexpr uint32_t PAUSE_MS = 100;
  static constexpr uint32_t GROUP_PAUSE_MS = 300;

  pinMode(LED_PIN, OUTPUT);

  // Morse "OK": O = long long long, K = long short long.
  const uint32_t pattern[][3] = {
      { LONG_MS, LONG_MS,  LONG_MS },
      { LONG_MS, SHORT_MS, LONG_MS },
  };

  for (size_t g = 0; g < sizeof(pattern) / sizeof(pattern[0]); ++g) {
    if (g > 0) delay(GROUP_PAUSE_MS);
    for (size_t i = 0; i < sizeof(pattern[0]) / sizeof(pattern[0][0]); ++i) {
      digitalWriteFast(LED_PIN, HIGH);
      delay(pattern[g][i]);
      digitalWriteFast(LED_PIN, LOW);
      delay(PAUSE_MS);
    }
  }
}

void setupInterruptPriorities() {
  // SPI first, then Ethernet, then SDIO last. SD reads happen in the main
  // loop (Modes 2/3/4), so the SDHC IRQ stays below SPI and Ethernet.
  NVIC_SET_PRIORITY(IRQ_LPSPI4, 0);   // Teensy 4.1 "SPI"  (B0)
  NVIC_SET_PRIORITY(IRQ_LPSPI3, 0);   // Teensy 4.1 "SPI1" (B1)
  NVIC_SET_PRIORITY(IRQ_ENET,   64);
  NVIC_SET_PRIORITY(IRQ_SDHC1,  96);  // USDHC1 drives the built-in SD slot
}
