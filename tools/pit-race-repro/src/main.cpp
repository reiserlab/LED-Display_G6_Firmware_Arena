// PIT race reproducer. Serial commands (115200, any baud):
//   u  : run the STOCK sequence   — loop { timer.end(); timer.begin(cb, period); }
//   g  : run the GUARDED sequence — same, but IRQ_PIT masked at the NVIC around end()
//   s  : stop the loop (timer left running)
//   ?  : print status
// Heartbeat every 500 ms: mode, end()/begin() cycles, callback ticks, PIT IRQ entries.
// A storm shows as: heartbeat stops, LED freezes. USB stays enumerated (the USB ISR
// shares priority 128 with the PIT and wins the tie-break), so the 134-baud bootloader
// reboot still works: teensy_loader_cli --mcu=TEENSY41 -b
#include <Arduino.h>
#include <IntervalTimer.h>

static IntervalTimer timer;
static volatile uint32_t ticks = 0;
static volatile uint32_t pit_irq_entries = 0;
static uint32_t cycles = 0;
static char mode = 's';
static const uint32_t PERIOD_US = 100;  // 10 kHz
static const uint32_t PERIOD_CYCLES = PERIOD_US * (F_CPU_ACTUAL / 1000000);  // 60 000 at 600 MHz
static const int32_t  SWEEP_CYCLES  = 1200;  // scan end() across expiry ± 2 µs, one cycle per iteration

static void cb() { ticks++; }

// Wrap the core's PIT vector so we can count raw PIT interrupt entries (the storm
// signature: entries explode while ticks stop, because pit_isr() skips a null callback).
static void (*saved_pit_isr)(void) = nullptr;
static void pit_tramp() { pit_irq_entries++; saved_pit_isr(); }
static void wrap_pit_vector() {
  void (*cur)(void) = _VectorsRam[IRQ_PIT + 16];
  if (cur == pit_tramp) return;
  saved_pit_isr = cur;
  attachInterruptVector(IRQ_PIT, pit_tramp);
}

static void arm() {
  timer.begin(cb, PERIOD_US);
  wrap_pit_vector();  // begin() re-attaches pit_isr, so re-wrap after every begin()
}

static void end_stock() { timer.end(); }
static void end_guarded() {
  NVIC_DISABLE_IRQ(IRQ_PIT);
  asm volatile("dsb\n\tisb" ::: "memory");
  timer.end();
  asm volatile("dsb" ::: "memory");
  NVIC_ENABLE_IRQ(IRQ_PIT);
}

static void status(const char* why) {
  Serial.printf("[%lu ms] %s mode=%c cycles=%lu ticks=%lu pit_irq=%lu TFLG0=%lu TCTRL0=%lu\n",
                millis(), why, mode, cycles, (unsigned long)ticks, (unsigned long)pit_irq_entries,
                (unsigned long)PIT_TFLG0, (unsigned long)PIT_TCTRL0);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  arm();
  Serial.println("pit-race-repro: u=stock g=guarded s=stop ?=status");
  status("boot");
}

void loop() {
  static uint32_t last_hb = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'u' || c == 'g' || c == 's') { mode = c; status("cmd"); }
    else if (c == '?') status("status");
  }
  if (mode == 'u' || mode == 'g') {
    // The disarm must land NEAR the timer's expiry, otherwise begin() keeps restarting the
    // period and the timer never fires (the display-starvation effect). Wait one period plus a
    // sweep offset (cycle-accurate), so successive iterations scan end() across the expiry.
    static int32_t offset = -SWEEP_CYCLES;
    arm();
    uint32_t start = ARM_DWT_CYCCNT;
    uint32_t target = PERIOD_CYCLES + offset;
    while ((uint32_t)(ARM_DWT_CYCCNT - start) < target) {}
    if (mode == 'u') end_stock(); else end_guarded();
    cycles++;
    if (++offset > SWEEP_CYCLES) offset = -SWEEP_CYCLES;
  }
  uint32_t now = millis();
  if (now - last_hb >= 500) {
    last_hb = now;
    digitalToggle(LED_BUILTIN);
    status("hb");
  }
}
