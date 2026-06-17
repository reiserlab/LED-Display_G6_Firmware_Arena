#include <Arduino.h>
#include "NetworkManager.h"
#include "SerialManager.h"
#include "SpiManager.h"
#include "SdManager.h"
#include "CommandProcessor.h"

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
#endif

void blinkStartupPattern();
void setupInterruptPriorities();

void setup() {
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
#endif

  // LED_BUILTIN is shared with SCK on SPI bus B0 (D13). Drive the boot blink
  // BEFORE SPI.begin() takes over the pin — never digitalWrite(LED_BUILTIN, ...)
  // after the SPI bus is up, or the SCK will glitch during traffic.
  blinkStartupPattern();

  net.begin();
  serial.begin();
  cmdProc.begin();
  spi.begin();
  sd.begin();  // mounts BUILTIN_SDCARD for Modes 2/3/4; safe with no card

  setupInterruptPriorities();
}

void loop() {
  net.serviceTcp();           // 1a. Accept TCP client, read and parse commands
  serial.serviceUsb();        // 1b. Read and parse commands from USB CDC
  cmdProc.processCommand();   // 2.  Handle one parsed command per source
  cmdProc.serviceDisplay();   // 3.  Re-transmit current frame at refresh rate
  net.flushResponses();       // 4a. Send queued responses over TCP
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
