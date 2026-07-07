#include "EthernetPhyDetect.h"

#include <Arduino.h>
#include <imxrt.h>

namespace {

// Pad/mux constants mirror QNEthernet's driver_teensy41.c exactly (same
// IOMUXC_PAD_* bitfields), so the PHY sees the identical strap/reset
// sequence it would from the real driver. Only the MDIO/MDC + strap +
// reset/power pins are needed for an ID read -- the RMII data pins and the
// PLL6/REFCLK setup are for the data path, not the management interface,
// and are skipped here.
constexpr uint32_t kGpioPadOutput = IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(7);
constexpr uint32_t kGpioMux = 5;  // ALT5 (GPIO)

constexpr uint32_t kStrapPadPullup = IOMUXC_PAD_PUS(3) | IOMUXC_PAD_PUE |
                                     IOMUXC_PAD_PKE | IOMUXC_PAD_SPEED(0) |
                                     IOMUXC_PAD_DSE(7);
constexpr uint32_t kStrapPadPulldown = IOMUXC_PAD_PUS(0) | IOMUXC_PAD_PUE |
                                       IOMUXC_PAD_PKE | IOMUXC_PAD_SPEED(0) |
                                       IOMUXC_PAD_DSE(7);
constexpr uint32_t kMdioPadPullup = IOMUXC_PAD_PUS(3) | IOMUXC_PAD_PUE |
                                    IOMUXC_PAD_PKE | IOMUXC_PAD_ODE |
                                    IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(5) |
                                    IOMUXC_PAD_SRE;
constexpr uint32_t kMdcPadPullup = IOMUXC_PAD_PUS(2) | IOMUXC_PAD_PUE |
                                   IOMUXC_PAD_PKE | IOMUXC_PAD_SPEED(3) |
                                   IOMUXC_PAD_DSE(5) | IOMUXC_PAD_SRE;
constexpr uint32_t kMdioMux = 0;  // ALT0

constexpr uint16_t kPhyIdr1 = 0x02;
constexpr uint16_t kPhyIdr2 = 0x03;

void configurePhyPins() {
  // Address/mode strap pins (PHY latches these on reset release).
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_04 = kStrapPadPulldown;  // PhyAdd[0] = 0
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_06 = kStrapPadPulldown;  // PhyAdd[1] = 0
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_05 = kStrapPadPullup;    // RMII Slave Mode
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_11 = kStrapPadPulldown;  // Auto MDIX Enable

  // Reset (RST_N) and power (PWRDN) pins, driven from GPIO7.
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_15 = kGpioPadOutput;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_14 = kGpioPadOutput;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_15 = kGpioMux;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_14 = kGpioMux;

  GPIO7_GDIR |= (1 << 15) | (1 << 14);
  GPIO7_DR_CLEAR = (1 << 15);  // Power down for now
  GPIO7_DR_SET   = (1 << 14);  // Reset de-asserted; assert for a fixed pulse below

  // MDIO / MDC pins.
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_15 = kMdioPadPullup;  // MDIO
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_14 = kMdcPadPullup;   // MDC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_15 = kMdioMux;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_14 = kMdioMux;
  IOMUXC_ENET_MDIO_SELECT_INPUT = 2;  // GPIO_B1_15_ALT0
}

// Same MMFR/EIR sequence as QNEthernet's mdio_read_nonblocking(), but with a
// hard timeout instead of an infinite spin -- a real PHY completes this in
// ~9us, so 1ms is generous headroom without meaningfully slowing boot.
bool mdioReadTimeout(uint16_t regaddr, uint16_t *data) {
  ENET_EIR = ENET_EIR_MII;  // Clear status
  ENET_MMFR = ENET_MMFR_ST(1) | ENET_MMFR_OP(2) | ENET_MMFR_PA(0 /*phyaddr*/) |
              ENET_MMFR_RA(regaddr) | ENET_MMFR_TA(2);

  const uint32_t start = micros();
  while ((ENET_EIR & ENET_EIR_MII) == 0) {
    if ((uint32_t)(micros() - start) > 1000) return false;
  }
  *data = ENET_MMFR_DATA(ENET_MMFR);
  ENET_EIR = ENET_EIR_MII;
  return true;
}

}  // namespace

bool ethernetPhyPopulated() {
  CCM_CCGR1 |= CCM_CCGR1_ENET(CCM_CCGR_ON);
  ENET_MSCR = ENET_MSCR_MII_SPEED(9);  // MDC from ipg_clk; no PLL6 needed for MDIO

  configurePhyPins();

  GPIO7_DR_SET   = (1 << 15);  // Power on
  delay(50);                   // Matches init_phy()'s "just in case" delay
  GPIO7_DR_CLEAR = (1 << 14);  // Reset
  delayMicroseconds(25);       // T1: minimum reset pulse width
  GPIO7_DR_SET   = (1 << 14);  // Take out of reset
  delay(2);                    // T2: reset-to-SMI-ready stabilization time

  // PHYIDR1/2 for the DP83825I: OUI bits give 0x2000 / (0x28xx & 0xfff0 == 0xA140).
  uint16_t idr1, idr2;
  const bool present = mdioReadTimeout(kPhyIdr1, &idr1) &&
                       mdioReadTimeout(kPhyIdr2, &idr2) &&
                       idr1 == 0x2000 && (idr2 & 0xfff0) == 0xA140;

  if (!present) {
    // Leave hardware in the same clean state init_phy() would on its own
    // no-hardware path: undo the GPIO outputs and gate the clock back off.
    GPIO7_GDIR &= ~((1u << 15) | (1u << 14));
    CCM_CCGR1 &= ~CCM_CCGR1_ENET(3);
  }
  // If present, leave clocks/pins as configured -- QNEthernet's own
  // Ethernet.begin() redoes this same setup from scratch (idempotent) and
  // will complete quickly now that a PHY is confirmed present to respond.

  return present;
}
