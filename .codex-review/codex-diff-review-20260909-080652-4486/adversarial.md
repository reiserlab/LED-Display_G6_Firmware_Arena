## Was This The Right Approach?

I would not merge this as-is. It combines three separable decisions into one firmware surface: changing Mode 4 gain semantics, adding board calibration, and changing the analog-read protocol. Those should be split.

A better approach would be:

- Keep `GET_ANALOG_IN_CMD` fixed at 4 bytes and add an extended read command for flags/calibrated metadata. The current change appends a fifth byte to `0xA4` while assuming old hosts ignore it (`src/CommandProcessor.cpp:806`, `src/constants.h:238`). That is only safe for length-tolerant parsers.
- Gate the new Mode 4 equation behind an explicit protocol/version negotiation or new mode behavior flag. This changes `fps` by 100x relative to the previous implementation (`src/CommandProcessor.cpp:1761`; old diff changed from `V * gain/10` to `V * 100 * gain/10`). That is not a calibration tweak; it is an experiment-behavior change.
- Treat calibration as an inactive-state operation with a transaction/commit step. The current `SET_ANALOG_CAL_CMD` mutates persisted calibration point-by-point immediately (`src/CommandProcessor.cpp:849`, `src/CommandProcessor.cpp:867`), even while playback or transfers may be active.

The current implementation wins on simplicity and controller autonomy. It trades away host compatibility, operational safety, and the ability to stage/validate calibration before it affects live closed-loop behavior.

## What Assumptions Does This Code Make, And Which Might Be Wrong?

It assumes appending a byte to `GET_ANALOG_IN_CMD` is backward-compatible. That is load-bearing and likely false for strict decoders. The tests now require exactly 5 bytes (`tests/test_io_roles.py:197`), while the implementation says pre-F1 hosts read 4 bytes and ignore the rest (`src/CommandProcessor.cpp:811`). If wrong, older host tools fail or desynchronize.

It assumes “open BNC equals +10 V” and “ground cap equals 0 V” are enough to calibrate every board (`src/constants.h:245`). If the input is still connected, the pull-up is weak relative to a source, the board variant differs, or the front-end is nonlinear/saturated, the firmware can store a plausible but wrong slope. The only validation is `raw_open > raw_gnd` and span >= 100 counts (`src/CommandProcessor.cpp:1687`). We would know only by comparing raw point distributions across boards or seeing Mode 4 drift/mis-scale in use.

It assumes calibration can apply immediately. `serviceClosedLoop()` reads `ain_cal_` on every sample (`src/CommandProcessor.cpp:1747`), and `SET_ANALOG_CAL_CMD` updates the same record in-place (`src/CommandProcessor.cpp:865`). If a user recalibrates or clears during a trial, the control law changes mid-trial.

It assumes EEPROM address 0 is unowned forever (`src/constants.h:254`). That may be true today, but it creates a silent future collision point for any other persisted controller config.

## Failure Modes The Code Does Not Address

Wrong calibration can overflow the reported mV format. A barely-valid span of 100 counts allows a 100 mV/count slope; `ainMv()` can return hundreds of volts equivalent (`src/CommandProcessor.cpp:1701`), then `GET_ANALOG_IN_CMD` casts to `int16_t` (`src/CommandProcessor.cpp:818`). That can wrap instead of reporting an error.

Power loss during `EEPROM.put()` can leave a bad CRC; boot then silently falls back to nominal scale except debug output (`src/CommandProcessor.cpp:1626`). That may make the same experiment run differently after a reboot.

Power loss or SD failure during mirror write deletes the previous JSON before opening the new one (`src/CommandProcessor.cpp:1670`). The authoritative EEPROM may survive, but the provenance mirror can be lost.

At 10x or 100x runtime scale, Mode 4’s unbounded per-frame stepping loop becomes risky. The new unity gain can request ±1000 fps (`src/constants.h:204`), then `serviceClosedLoop()` advances one frame per loop iteration in `while` loops (`src/CommandProcessor.cpp:1764`). A delayed loop or high voltage can produce a large catch-up burst.

Two clients can interleave calibration actions. `processCommand()` handles network and serial independently (`src/CommandProcessor.cpp:114`), while calibration has no session ownership or transaction boundary. Mixed open/ground samples from different operators can produce a valid-but-wrong record.

## Hidden Costs

This adds persistent state, host protocol changes, analog calibration policy, and control-loop filtering all inside `CommandProcessor` (`src/CommandProcessor.h:89`). That class is already responsible for display modes, SD files, networking responses, AO, DIO, firmware update, and pattern playback. Debugging future behavior now requires knowing EEPROM contents, SD mirror state, host capability handling, and live analog state.

The calibration record is a packed C++ struct used directly as EEPROM layout (`src/CommandProcessor.h:91`). That is compact, but it makes schema evolution brittle. Any field reorder, type change, or endianness assumption becomes a persisted-state migration.

The tests also carry operational cost. `test_deadband_round_trip` writes real EEPROM on every default run (`tests/test_analog_cal.py:76`). It restores the old value, but a test interruption leaves changed board state, and repeated CI/bench runs consume EEPROM write budget.

## Reversibility

This is not treated like a one-way door, but it is one.

Public opcodes `0xA5`, `0xA6`, and `0xA7` are added (`src/commands.h:47`), capability bit 6 is advertised (`src/constants.h:151`), and `0xA4` response shape changes (`src/commands.h:42`). Six months from now, undoing this requires coordinated host changes and possibly field cleanup of EEPROM records.

The Mode 4 formula change is especially hard to unwind because trial files or host presets may be adjusted around the new 100 fps/V unity convention (`README.md:98`, `src/constants.h:197`). Once users retune gains, reverting the firmware silently breaks experiments in the opposite direction.

EEPROM records at address 0 persist across firmware flashes (`src/constants.h:254`). A rollback firmware that does not understand them will ignore them, while a future firmware may accidentally reuse the same space unless there is a central allocation registry.

## Race Conditions, Data-Loss Risks, Rollback Risks, Reliability Risks

Race condition: live Mode 4 reads `ain_cal_` while commands can mutate it in the same single-threaded event loop between samples (`src/CommandProcessor.cpp:849`, `src/CommandProcessor.cpp:1747`). That avoids memory races, but not behavioral races: a trial can change scale halfway through.

Data-loss risk: SD mirror write removes the previous file before the replacement is durable (`src/CommandProcessor.cpp:1671`). Use temp-write plus rename if the mirror matters.

Rollback risk: old firmware after rollback will not know about the EEPROM record; new host software may continue assuming bit 6 / 5-byte `0xA4` semantics. There is no migration or “clear all calibration” command, only per-channel clear (`src/CommandProcessor.cpp:889`).

Reliability risk: calibration writes are not guarded against active display state or SD transfers, unlike pattern-start and purge paths (`src/CommandProcessor.cpp:1118`, `src/CommandProcessor.cpp:512`). `SET_ANALOG_CAL_CMD` can perform ADC sampling, EEPROM write, and SD file operations while playback or archive/upload/download operations are in progress (`src/CommandProcessor.cpp:867`, `src/CommandProcessor.cpp:1654`, `src/CommandProcessor.cpp:1667`).

## The Strongest Argument Against Merging

The strongest argument against merging is that this makes experimental behavior depend on mutable board-local state and changes the Mode 4 gain scale by 100x in the same patch, without a hard protocol/version boundary or inactive-only calibration workflow. That is exactly the kind of firmware change that can pass bench shape tests and still invalidate data because the controller “worked” while applying the wrong physical mapping.

If this is urgent, I would merge only the raw-read command and capability bit first. Then land calibrated Mode 4 behind explicit host support, stricter calibration validation, atomic SD mirroring, no active-trial mutation, and a deliberate migration story for the gain-scale change.