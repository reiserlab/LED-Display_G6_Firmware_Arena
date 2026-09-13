#!/bin/zsh
# Browser-free SD soak campaign: alternate patterns in 10-min Mode-3 segments with sd_stall_test.py,
# first at a high rate (stress phase), then at the course rate until a stop time. Survives a controller
# watchdog reboot (waits for the CDC port before each segment) and a failed segment (logged, continues).
#
#   scripts/sd_soak_campaign.sh PORT "IDX1 IDX2 ..." STRESS_HZ STRESS_UNTIL_HHMM NIGHT_HZ NIGHT_UNTIL_HHMM [SEG_MIN]
#   e.g. scripts/sd_soak_campaign.sh /dev/cu.usbmodem121699401 "37 36" 286 1815 200 0800 10
#
# Logs: soak-logs/sdstall-*-camp-<hz>-p<idx>.jsonl + soak-logs/campaign-<start>.log (one line per segment).
# Report: python3 webDisplayTools/scripts/telemetry-report.py soak-logs/sdstall-*-camp-*.jsonl
PORT=$1; PATS=(${=2}); SHZ=$3; SUNTIL=$4; NHZ=$5; NUNTIL=$6; SEG=${7:-10}
cd "$(dirname "$0")/.." || exit 2
mkdir -p soak-logs
LOG=soak-logs/campaign-$(date +%Y%m%d-%H%M%S).log
now() { TZ=America/New_York date '+%H:%M:%S ET'; }
hhmm() { TZ=America/New_York date '+%H%M'; }
# "until" times are compared on the ET clock; a NIGHT_UNTIL before the current time means "tomorrow".
phase_over() { local until=$1 started=$2; local h=$(hhmm)
  if (( until > started )); then (( h >= until || h < started - 100 )); else (( h >= until && h < started )); fi; }
echo "$(now) campaign start: patterns ${PATS[*]}, stress ${SHZ} Hz until ${SUNTIL}, night ${NHZ} Hz until ${NUNTIL}, ${SEG}-min segments" | tee -a "$LOG"
i=0
run_phase() { local hz=$1 until=$2 tag=$3; local started=$(hhmm)
  while ! phase_over $until $started; do
    local idx=${PATS[$(( i % ${#PATS[@]} + 1 ))]}; i=$((i+1))
    local n=0; until [[ -e $PORT ]]; do sleep 2; n=$((n+1)); (( n < 150 )) || { echo "$(now) port $PORT gone for 5 min — giving up" | tee -a "$LOG"; return 1; }; done
    echo "$(now) segment $i: pattern $idx at ${hz} Hz for ${SEG} min ($tag)" | tee -a "$LOG"
    python3 scripts/sd_stall_test.py --port "$PORT" --pattern "$idx" --hz "$hz" --minutes "$SEG" --sd-diag 0 \
        --label "camp-${hz}-p${idx}" --log-dir soak-logs > "soak-logs/segment-$i.out" 2>&1
    local rc=$?
    local summ=$(grep -o '"stalls_over_gap": [0-9]*, "stall_detail_truncated": [a-z]*, "clusters": [0-9]*' soak-logs/segment-$i.out | head -1)
    local worst=$(grep -o '"worst_ms": [0-9.]*' soak-logs/segment-$i.out | head -1)
    local usable=$(grep -o '"measurement_usable": [a-z]*' soak-logs/segment-$i.out | head -1)
    echo "$(now) segment $i done rc=$rc $summ $worst $usable" | tee -a "$LOG"
    (( rc == 0 )) || sleep 10
  done; }
run_phase "$SHZ" "$SUNTIL" stress
run_phase "$NHZ" "$NUNTIL" night
echo "$(now) campaign end after $i segments" | tee -a "$LOG"
