# Opcode constants mirroring src/commands.h (AC::ArenaCommands enum).
# Keep in sync with the C source; this is the single Python source of truth.

ALL_OFF_CMD                 = 0x00
SYSTEM_RESET_CMD            = 0x01   # software system reset (SCB_AIRCR SYSRESETREQ)
SWITCH_GRAYSCALE_CMD        = 0x06   # dropped for G6 — firmware returns status=1
TRIAL_PARAMS_CMD            = 0x08
SET_REFRESH_RATE_CMD        = 0x16
GET_REFRESH_RATE_CMD        = 0x17   # returns uint16 LE Hz
STOP_DISPLAY_CMD            = 0x30
STREAM_FRAME_CMD            = 0x32
GET_FRAMES_SENT_CMD         = 0x33   # returns uint32 LE
RESET_FRAMES_SENT_CMD       = 0x34
SET_FRAME_POSITION_CMD      = 0x70
GET_FILE_COUNT_CMD          = 0x80   # returns uint16 LE
GET_PATTERN_FILENAME_CMD    = 0x82
SET_PATTERN_FILENAME_CMD    = 0x83
GET_PATTERN_FILE_CMD        = 0x84
SET_PATTERN_FILE_CMD        = 0x85
DELETE_PATTERN_FILE_CMD     = 0x86
GET_SD_ARCHIVE_CMD          = 0x8A
DELETE_ALL_PATTERNS_CMD     = 0x8F
SET_AO_VOLTAGE_CMD          = 0xA0   # [03 A0 mv_lo mv_hi] set BNC J27 (MCP4725) 0-5000 mV
GET_AO_VOLTAGE_CMD          = 0xA1   # [01 A1] returns hardware DAC readback as uint16 LE mV
SET_DIGITAL_OUT_CMD         = 0xAA   # [03 AA ch state] DO1 (ch=1, J3) or DO2 (ch=2, J4)
GET_DIGITAL_OUT_CMD         = 0xAB   # [01 AB] returns DO1 and DO2 state as two bytes
SET_ETHERNET_IP_ADDRESS_CMD = 0xC0   # reserved, not yet implemented
GET_ETHERNET_IP_ADDRESS_CMD = 0xC1
GET_CONTROLLER_INFO_CMD     = 0xC2   # returns {version, capability_bitmap}
SET_DIAG_OUTPUT_CMD         = 0xC3
GET_DIAG_OUTPUT_CMD         = 0xC4   # returns 0 or 1
SET_SPI_CLOCK_CMD           = 0xC5   # [len=3,0xC5,lo,hi] uint16 LE MHz; echoes applied MHz
GET_SPI_CLOCK_CMD           = 0xC6   # returns uint16 LE MHz
ALL_ON_CMD                  = 0xFF
