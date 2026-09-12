# Opcode constants mirroring src/commands.h (AC::ArenaCommands enum).
# Keep in sync with the C source; this is the single Python source of truth.

ALL_OFF_CMD                 = 0x00
SYSTEM_RESET_CMD            = 0x01   # software system reset (SCB_AIRCR SYSRESETREQ)
SET_PATTERN_ID_CMD          = 0x03   # [03 03 id_lo id_hi] load 1-based pattern into Mode 3 at frame 0
SWITCH_GRAYSCALE_CMD        = 0x06   # dropped for G6 — firmware returns status=1
TRIAL_PARAMS_CMD            = 0x08   # Modes 2/3/4; param[10] = per-trial duty, 0 = pattern's stored (#33)
SET_REFRESH_RATE_CMD        = 0x16
GET_REFRESH_RATE_CMD        = 0x17   # returns uint16 LE Hz
SET_PANEL_DISPLAY_MODE_CMD  = 0x1B   # [02 1B mode] 0=oneshot 1=persist 2=triggered 3=gated; default=persist
GET_PANEL_DISPLAY_MODE_CMD  = 0x1C   # [01 1C] returns current panel display mode as single byte
STOP_DISPLAY_CMD            = 0x30
STREAM_FRAME_CMD            = 0x32
GET_FRAMES_SENT_CMD         = 0x33   # returns uint32 LE
RESET_FRAMES_SENT_CMD       = 0x34
SET_FRAME_POSITION_CMD      = 0x70
GET_FRAME_POSITION_CMD      = 0x72   # [01 72] returns cur_frame_index + frame_count, both uint16 LE
GET_FILE_COUNT_CMD          = 0x80   # returns uint16 LE
GET_PATTERN_FILENAME_CMD    = 0x82
GET_PATTERN_INFO_CMD        = 0x88   # [03 88 idx_lo idx_hi] 1-based; framed reply: header metadata + frame-0 duty_cycle
SET_PATTERN_FILENAME_CMD    = 0x83
GET_PATTERN_FILE_CMD        = 0x84
SET_PATTERN_FILE_CMD        = 0x85
DELETE_PATTERN_FILE_CMD     = 0x86
GET_SD_ARCHIVE_CMD          = 0x8A
PURGE_MEMORY_CMD            = 0x8F   # formats the SD card (wipes everything, not just /patterns)
SET_FIRMWARE_FILE_CMD       = 0xE0   # [0xE0, len_b0..b7, data...] upload image -> /firmware/panel.bin; reply u32 LE CRC-32
SET_AO_VOLTAGE_CMD          = 0xA0   # [03 A0 mv_lo mv_hi] set BNC J27 (MCP4725) 0-5000 mV
GET_AO_VOLTAGE_CMD          = 0xA1   # [01 A1] returns hardware DAC readback as uint16 LE mV
SET_AO_LUT_CMD              = 0xA2   # [len A2 mode step_hz_lo step_hz_hi count_lo count_hi mv...] upload+start AO LUT
SET_AO_MODE_CMD             = 0xA3   # [02 A3 mode] 0=programmable | 1=frame_number (DAC tracks frame index, 0-5V)
GET_ANALOG_IN_CMD           = 0xA4   # [01 A4] returns Analog In 1+2 as two int16 LE mV (±10V front-end)
SET_TELEMETRY_CMD           = 0xA8   # [04 A8 flags rate_lo rate_hi] or [02 A8 flags]; bit0 = record events (default ON), bit7 = synthetic producer at rate/s; bit4 = watchdog bits present (bit6 off, bit5 starve)
GET_TELEMETRY_BLOCK_CMD     = 0xA9   # [08 A9 ack_seq(u32) max_bytes(u16) flags] ack cursor + framed chunk of ring records; see telemetry_codec.py
SET_DIGITAL_OUT_CMD         = 0xAA   # [03 AA ch state] drive "Digital IO 1/2 (5V)" BNC (role-gated, #135)
GET_DIGITAL_OUT_CMD         = 0xAB   # [01 AB] returns Digital IO 1 and 2 data-pin state as two bytes
SET_DIO_ROLE_CMD            = 0xAC   # [03 AC port role] 0=off 1=in_trigger 2=out_programmable 3=out_debug_framescan
GET_DIO_ROLE_CMD            = 0xAD   # [01 AD] returns [role1, level1, role2, level2]
SET_ETHERNET_IP_ADDRESS_CMD = 0xC0   # reserved, not yet implemented
GET_ETHERNET_IP_ADDRESS_CMD = 0xC1
GET_CONTROLLER_INFO_CMD     = 0xC2   # returns {version, capability_bitmap, mac[6]} (bit5 = io_ext, bit7 = health)
SET_DIAG_OUTPUT_CMD         = 0xC3
GET_DIAG_OUTPUT_CMD         = 0xC4   # returns 0 or 1
SET_SPI_CLOCK_CMD           = 0xC5   # [len=3,0xC5,lo,hi] uint16 LE MHz; echoes applied MHz
GET_SPI_CLOCK_CMD           = 0xC6   # returns uint16 LE MHz
GET_HEALTH_CMD              = 0xCA   # [01 CA] read-only health telemetry + reset-surviving breadcrumb (issue #50); 66-byte LE payload, see test_health.py
GET_CRASHREPORT_CMD         = 0xCC   # [01 CC] raw 128 B PJRC CrashReport region (0x2027FF80..); gate on 0xCB flags bit 2
GET_FIRMWARE_VERSION_CMD    = 0xCB   # [01 CB] build identity {ver, rows, cols, flags, sha[8], date[10], branch[24]} = 46 bytes; see test_firmware_version.py
ALL_ON_CMD                  = 0xFF
