## Correctness

**Blocking**
- None found.

**Significant**
- `src/CommandProcessor.cpp:1687` validates a calibration with only `raw_open > raw_gnd` and a 100-count span. That can mark a mis-sampled calibration valid, after which `src/CommandProcessor.cpp:1701` can produce mV values far outside the `int16` response range used at `src/CommandProcessor.cpp:818`, wrapping the host-visible voltage and driving huge Mode 4 velocities at `src/CommandProcessor.cpp:1761`. Require plausible raw ranges/span for the expected 0 V and +10 V points, or clamp/refuse calibrated mV values that cannot fit the protocol.
- `SET_ANALOG_CAL_CMD` can run while a display is active (`src/CommandProcessor.cpp:849`). For Mode 4, the same record is read in the control loop (`src/CommandProcessor.cpp:1747` and `src/CommandProcessor.cpp:1756`), so calibration can change mid-trial and persist bad samples taken with the experiment signal connected. Refuse calibration writes unless `state_ == ArenaState::ALL_OFF`.

**Minor**
- New read-only commands do not enforce their documented empty payload shape: `GET_ANALOG_IN_RAW_CMD` at `src/CommandProcessor.cpp:832` and `GET_ANALOG_CAL_CMD` at `src/CommandProcessor.cpp:843` ignore extra bytes. `SET_ANALOG_CAL_CMD` also accepts trailing bytes for sample/clear and extra trailing bytes for deadband (`src/CommandProcessor.cpp:855`, `src/CommandProcessor.cpp:877`). Tighten these to exact lengths so protocol misuse is caught.

## Tests

**Blocking**
- None.

**Significant**
- The Mode 4 behavioral change is essentially untested. The code changed the closed-loop formula, added EWMA smoothing, and added deadband behavior at `src/CommandProcessor.cpp:1747`-`src/CommandProcessor.cpp:1762`, but the existing gain test explicitly only checks command acceptance (`tests/test_gh4_trial_params_gain_duration.py:17`). Add a test that drives or stubs a known AIN value and verifies frame-position advancement, deadband stop, and negative gain direction.
- No test covers rejecting calibration while display/Mode 4 is active, or rejecting implausible two-point calibration records. Those are the main safety paths for this change.

**Minor**
- `test_deadband_round_trip` is described as non-destructive, but it writes EEPROM and the SD mirror on every normal test run (`tests/test_analog_cal.py:76`). That is not data-destructive, but it is persistent state mutation and EEPROM wear. Gate it like the sampling test or move it behind an explicit hardware-state fixture.

## Fit With Existing Code

**Blocking**
- None.

**Significant**
- Existing SD/display mutations are guarded around active operation and transfers, but analog calibration is not guarded against display-active state. This breaks the local pattern used by `TRIAL_PARAMS_CMD` (`src/CommandProcessor.cpp:1118`) and other state-changing SD/display commands.

**Minor**
- `scripts/controller_info.py:17`-`scripts/controller_info.py:24` still decodes only capability bits 0-4, so it will omit both `io_ext` and the newly advertised `ai_cal` bit even though `src/constants.h:155` now reports `0x63`. Update the script’s capability table.

## Risk And Reliability

**Blocking**
- None.

**Significant**
- The new G3-faithful `100 fps/V` multiplier increases worst-case frame accumulation by 100x (`src/CommandProcessor.cpp:1761`). With accepted gains such as 2000 (`tests/test_gh4_trial_params_gain_duration.py:56`) and high/floating analog input, the `while` loops at `src/CommandProcessor.cpp:1765` and `src/CommandProcessor.cpp:1770` can spin hundreds or thousands of iterations per service tick. Replace the loops with a bounded whole-step/modulo calculation or cap the effective per-tick advance.

**Minor**
- I did not run the tests/build because the workspace is read-only and build/test commands would write artifacts.

## Specific Suggested Changes

**Blocking**
- None.

**Significant**
- Add `state_ == ArenaState::ALL_OFF` validation at the start of `SET_ANALOG_CAL_CMD` in `src/CommandProcessor.cpp:849`.
- Strengthen `ainCalValidate` at `src/CommandProcessor.cpp:1687` to reject implausible raw points, and guard/clamp `ainMv` conversion before packing into `int16` at `src/CommandProcessor.cpp:818`.
- Replace the Mode 4 per-frame `while` stepping at `src/CommandProcessor.cpp:1765`-`src/CommandProcessor.cpp:1774` with bounded modulo arithmetic.

**Minor**
- Enforce exact command lengths for `0xA5`, `0xA6`, and `0xA7`.
- Update `scripts/controller_info.py:17` to include bit 5 `io_ext` and bit 6 `ai_cal`.
- Add HIL or unit coverage for calibrated Mode 4 frame advancement, deadband, and bad calibration rejection.