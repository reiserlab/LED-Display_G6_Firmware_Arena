#include <Arduino.h>
#include "NetworkManager.h"
#include "SpiManager.h"
#include "CommandProcessor.h"

NetworkManager   net;
SpiManager       spi;
CommandProcessor cmdProc(net, spi);

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
  cmdProc.begin();
  spi.begin();

  setupInterruptPriorities();
}

void loop() {
  net.serviceTcp();         // 1. Accept client, read and parse commands
  cmdProc.processCommand(); // 2. Handle one parsed command
  cmdProc.serviceDisplay(); // 3. Re-transmit current frame at refresh rate
  net.flushResponses();     // 4. Send queued responses over TCP

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
  // SPI first, then Ethernet. SDIO not enabled in this build.
  NVIC_SET_PRIORITY(IRQ_LPSPI4, 0);   // Teensy 4.1 "SPI"  (B0)
  NVIC_SET_PRIORITY(IRQ_LPSPI3, 0);   // Teensy 4.1 "SPI1" (B1)
  NVIC_SET_PRIORITY(IRQ_ENET,   64);
}
