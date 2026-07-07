#pragma once

// Probes for the Teensy 4.1's optional DP83825I Ethernet PHY without going
// through QNEthernet. QNEthernet's own presence check (driver_teensy41.c
// init_phy() -> mdio_read()) spins forever waiting for a hardware "MDIO
// transaction complete" flag that never sets when there is no PHY on the bus
// to finish the transaction -- so on any board where the PHY isn't
// populated, calling Ethernet.begin() hangs the whole MCU, including USB.
// This duplicates just enough of init_phy()'s register sequence (clocks,
// pin mux, PHY reset pulse, PHY ID read) to answer the same question, but
// with a bounded timeout on the MDIO read instead of an infinite spin.
bool ethernetPhyPopulated();
