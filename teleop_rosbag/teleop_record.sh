#!/usr/bin/env bash
# 기록을 켜고 텔레옵을 한다. 텔레옵을 끝내면 기록도 닫는다.
#   ./teleop_record.sh -t cal_cam                 # 태그 cal_cam 으로 기록하면서 텔레옵
#   ./teleop_record.sh -t laser -- --allow-fire   # -- 뒤는 teleop_keys.py 인자
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TAG="untagged"; ROOT="${BAG_ROOT:-$HOME/bags}"; EXCLUDE=""
while getopts "t:o:x:h" opt; do
  case "$opt" in
    t) TAG="$OPTARG" ;; o) ROOT="$OPTARG" ;; x) EXCLUDE="$OPTARG" ;;
    h) sed -n '2,4p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))
case "$TAG" in *[!A-Za-z0-9_-]*) echo "태그는 영문·숫자·_·- 만 쓴다: $TAG"; exit 2 ;; esac
SESSION="$ROOT/$(date +%Y%m%d_%H%M%S)_${TAG}_$(hostname -s 2>/dev/null || echo host)"
REC=(bash "$HERE/record.sh" -q -t "$TAG" -S "$SESSION"); [ -n "$EXCLUDE" ] && REC+=(-x "$EXCLUDE")
"${REC[@]}" &
RPID=$!
stop_record() {
  if kill -0 "$RPID" 2>/dev/null; then
    echo "기록을 닫는 중… (최대 20초)"
    kill -TERM "$RPID" 2>/dev/null
    wait "$RPID" 2>/dev/null
  fi
}
trap stop_record EXIT
trap ':' INT
trap 'exit 130' TERM HUP
echo "기록 준비 중…"
for _ in $(seq 1 20); do
  [ -f "$SESSION/rec_status" ] && break
  kill -0 "$RPID" 2>/dev/null || { echo "기록을 시작하지 못했다. 텔레옵을 하지 않는다."; [ -f "$SESSION/record.log" ] && tail -3 "$SESSION/record.log"; exit 2; }
  sleep 1
done
grep -q '^RECORDING' "$SESSION/rec_status" 2>/dev/null || { echo "기록을 시작하지 못했다: $(cat "$SESSION/rec_status" 2>/dev/null)"; exit 2; }
echo "기록 중: $SESSION"
python3 "$HERE/teleop_keys.py" --rec-status-file "$SESSION/rec_status" --event-log "$SESSION/teleop_events.jsonl" "$@"
TRC=$?
stop_record
trap - EXIT
echo "기록 폴더: $SESSION"
[ -f "$SESSION/record.log" ] && grep -E "경고|없음|부족" "$SESSION/record.log" | tail -8
echo "점검 결과는  python3 $HERE/bag_check.py $SESSION  로 다시 볼 수 있다."
exit "$TRC"
