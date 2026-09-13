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
# Deadlines are resolved ONCE per phase to an absolute epoch: HHMM today on the ET clock, or tomorrow if
# that instant has already passed (an overnight NIGHT_UNTIL). No sampling of a narrow clock window.
deadline_epoch() { local hhmm=$1; local d
  d=$(TZ=America/New_York date -j -f '%Y-%m-%d %H%M' "$(TZ=America/New_York date '+%Y-%m-%d') $hhmm" '+%s' 2>/dev/null) || return 1
  (( d <= $(date +%s) )) && d=$(( d + 86400 ))
  echo $d; }
echo "$(now) campaign start: patterns ${PATS[*]}, stress ${SHZ} Hz until ${SUNTIL}, night ${NHZ} Hz until ${NUNTIL}, ${SEG}-min segments" | tee -a "$LOG"
i=0; failed=0
run_phase() { local hz=$1 tag=$3; local until; until=$(deadline_epoch "$2") || { echo "$(now) bad deadline $2" | tee -a "$LOG"; return 1; }
  echo "$(now) phase $tag: ${hz} Hz until $(TZ=America/New_York date -r $until '+%Y-%m-%d %H:%M ET')" | tee -a "$LOG"
  while (( $(date +%s) < until )); do
    local remain_min=$(( (until - $(date +%s) + 59) / 60 )); local mins=$SEG; (( remain_min < mins )) && mins=$remain_min
    (( mins < 2 )) && break   # a segment shorter than 2 min is not a measurement
    local idx=${PATS[$(( i % ${#PATS[@]} + 1 ))]}; i=$((i+1))
    local n=0; until [[ -e $PORT ]]; do sleep 2; n=$((n+1)); (( n < 150 )) || { echo "$(now) port $PORT gone for 5 min — giving up" | tee -a "$LOG"; return 1; }; done
    echo "$(now) segment $i: pattern $idx at ${hz} Hz for ${mins} min ($tag)" | tee -a "$LOG"
    python3 scripts/sd_stall_test.py --port "$PORT" --pattern "$idx" --hz "$hz" --minutes "$mins" --sd-diag 0 \
        --label "camp-${hz}-p${idx}" --log-dir soak-logs > "soak-logs/segment-$i.out" 2>&1
    local rc=$?
    local summ=$(grep -o '"stalls_over_gap": [0-9]*, "stall_detail_truncated": [a-z]*, "clusters": [0-9]*' soak-logs/segment-$i.out | head -1)
    local worst=$(grep -o '"worst_ms": [0-9.]*' soak-logs/segment-$i.out | head -1)
    local usable=$(grep -o '"measurement_usable": [a-z]*' soak-logs/segment-$i.out | head -1)
    echo "$(now) segment $i done rc=$rc $summ $worst $usable" | tee -a "$LOG"
    (( rc == 0 )) || { failed=$((failed+1)); sleep 10; }
  done; }
rc_all=0
run_phase "$SHZ" "$SUNTIL" stress || rc_all=1
run_phase "$NHZ" "$NUNTIL" night  || rc_all=1
echo "$(now) campaign $([[ $rc_all == 0 ]] && echo completed || echo ABORTED) after $i segments ($failed with a non-zero exit)" | tee -a "$LOG"
exit $rc_all
