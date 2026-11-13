#!/usr/bin/env bash
# 모든 토픽을 rosbag 으로 기록한다.
#   ./record.sh -t cal_cam            # ~/bags/<날짜_시각>_cal_cam_<호스트>/ 에 기록. Ctrl-C 로 끝낸다
#   ./record.sh -t drive -x '/oak/.*' # 정규식에 맞는 토픽은 빼고 기록
# 안쪽 명령은  ros2 bag record -a -s mcap --max-bag-duration 60 -o <세션>/bag  이다.
# 기록 폴더(세션) 구성: bag/(rosbag) · record.log · notes.txt · info.txt · rec_status · teleop_events.jsonl(텔레옵과 같이 쓸 때)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TAG="untagged"; ROOT="${BAG_ROOT:-$HOME/bags}"; SESSION=""; QUIET=0; EXCLUDE=""
MIN_FREE_GB="${MIN_FREE_GB:-10}"; STOP_FREE_GB="${STOP_FREE_GB:-3}"; SPLIT_S="${SPLIT_S:-60}"
PROJECT="${MAICON_PROJECT:-$HERE/../maicon2026}"

usage() { sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'; echo "옵션: -t 태그  -o 기록 폴더 위치(기본 ~/bags)  -S 세션 폴더(직접 지정)  -x 제외 정규식  -q 화면 출력 줄이기"; }
while getopts "t:o:S:x:qh" opt; do
  case "$opt" in
    t) TAG="$OPTARG" ;; o) ROOT="$OPTARG" ;; S) SESSION="$OPTARG" ;; x) EXCLUDE="$OPTARG" ;; q) QUIET=1 ;;
    h) usage; exit 0 ;; *) usage; exit 2 ;;
  esac
done
case "$TAG" in *[!A-Za-z0-9_-]*) echo "태그는 영문·숫자·_·- 만 쓴다: $TAG"; exit 2 ;; esac
[ -n "$SESSION" ] || SESSION="$ROOT/$(date +%Y%m%d_%H%M%S)_${TAG}_$(hostname -s 2>/dev/null || echo host)"
BAG="$SESSION/bag"; LOG="$SESSION/record.log"; STATUS="$SESSION/rec_status"

say() { [ "$QUIET" = 1 ] || echo "$*"; [ -d "$SESSION" ] && echo "$(date +%H:%M:%S) $*" >> "$LOG"; return 0; }
die() { echo "record.sh: $*" >&2; [ -d "$SESSION" ] && echo "STOPPED: $*" > "$STATUS"; exit 2; }
free_gb() { df -Pk "$1" 2>/dev/null | awk 'NR==2 {printf "%d", $4/1048576}'; }
size_mb() { du -sm "$BAG" 2>/dev/null | awk '{print $1+0}'; }

# ---- 사전 점검
command -v ros2 >/dev/null 2>&1 || die "ros2 명령이 없다. source /opt/ros/humble/setup.bash 를 먼저 한다"
mkdir -p "$SESSION" || die "기록 폴더를 만들 수 없다: $SESSION"
[ -e "$BAG" ] && die "이미 기록이 있는 폴더다: $BAG"
say "기록 폴더: $SESSION"
say "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0(기본)}  ROS_DISTRO=${ROS_DISTRO:-?}"
FREE="$(free_gb "$SESSION")"; FREE="${FREE:-0}"
[ "$FREE" -ge "$MIN_FREE_GB" ] || die "남은 디스크 ${FREE}GB < ${MIN_FREE_GB}GB. 시작하지 않는다"
TOPICS="$(timeout 5 ros2 topic list 2>/dev/null)"
if [ -z "$TOPICS" ]; then
  say "토픽이 안 보인다. ros2 daemon 을 다시 띄운다"
  ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
  TOPICS="$(timeout 8 ros2 topic list 2>/dev/null)"
fi
[ -n "$TOPICS" ] || die "토픽이 하나도 없다. 로봇 드라이버가 떠 있는지, ROS_DOMAIN_ID 가 같은지 확인한다"
NT="$(echo "$TOPICS" | wc -l | tr -d ' ')"
say "토픽 ${NT}개"; echo "$TOPICS" >> "$LOG"
echo "$TOPICS" | grep -qx '/image_raw' || say "경고: /image_raw 가 안 보인다"
echo "$TOPICS" | grep -qx '/odom' || say "경고: /odom 이 안 보인다"

# ---- 버전마다 다른 옵션은 도움말에 있을 때만 쓴다
HELP="$(ros2 bag record --help 2>&1)"
has() { printf '%s' "$HELP" | grep -q -- "$1"; }
STORAGE="sqlite3"
if has "mcap"; then STORAGE="mcap"; else say "mcap 저장 플러그인이 없어 sqlite3 로 기록한다 (sudo apt install ros-humble-rosbag2-storage-mcap)"; fi
ARGS=(bag record -a -s "$STORAGE" -o "$BAG")
has "--max-bag-duration" && ARGS+=(--max-bag-duration "$SPLIT_S")
has "--max-cache-size" && ARGS+=(--max-cache-size 209715200)
[ "$STORAGE" = mcap ] && has "--storage-preset-profile" && ARGS+=(--storage-preset-profile fastwrite)
if [ -n "$EXCLUDE" ]; then has "--exclude" && ARGS+=(--exclude "$EXCLUDE") || say "경고: 이 버전에는 --exclude 가 없다. 전부 기록한다"; fi

# ---- 메모 (환경변수 전체는 남기지 않는다. 임무코드가 들어 있을 수 있다)
{
  echo "시작: $(date '+%Y-%m-%d %H:%M:%S %z')"
  echo "태그: $TAG"
  echo "호스트: $(hostname 2>/dev/null)"
  echo "저장 형식: $STORAGE"
  echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0(기본)}"
  echo "명령: ros2 ${ARGS[*]}"
  if [ -d "$PROJECT" ]; then
    C="$(git -C "$PROJECT" rev-parse --short HEAD 2>/dev/null)"; echo "코드 커밋: ${C:-git 저장소 아님}"
    for f in config/waveshare_real.json config/field_overrides.json; do
      [ -f "$PROJECT/$f" ] && echo "설정 해시 $f: $(sha256sum "$PROJECT/$f" | cut -c1-16)"
    done
  fi
  echo "메모(직접 적기): "
} > "$SESSION/notes.txt"

# ---- 실행. 새 세션에서 띄워 터미널의 Ctrl-C 가 기록기에 두 번 가지 않게 하고, 표준 입력은 닫는다
PRE=(); command -v nice >/dev/null 2>&1 && PRE+=(nice -n 10); command -v ionice >/dev/null 2>&1 && PRE+=(ionice -c2 -n7)
python3 -c 'import os, signal, sys
try:
    os.setsid()
except OSError:
    pass
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])' \
  "${PRE[@]}" ros2 "${ARGS[@]}" </dev/null >>"$LOG" 2>&1 &
PID=$!
STOPPING=0; WHY="사용자 종료"
stop() { STOPPING=1; }
trap stop INT TERM HUP
sleep 2
kill -0 "$PID" 2>/dev/null || die "기록기가 바로 끝났다. $LOG 를 본다"
echo "RECORDING" > "$STATUS"
say "기록 시작 (끝내려면 Ctrl-C)"

# ---- 감시: 10초마다 크기와 남은 디스크
T=0; LAST=0
while [ "$STOPPING" = 0 ]; do
  sleep 1; T=$((T + 1))
  kill -0 "$PID" 2>/dev/null || { WHY="기록기가 스스로 끝났다"; break; }
  if [ $((T % 10)) = 0 ]; then
    NOW="$(size_mb)"; NOW="${NOW:-0}"; FREE="$(free_gb "$SESSION")"; FREE="${FREE:-0}"
    RATE=$(( (NOW - LAST) * 6 )); LAST="$NOW"
    LEFT="-"; [ "$RATE" -gt 0 ] && LEFT="$(( FREE * 1024 / RATE ))분"
    say "기록 ${T}초  ${NOW}MB  분당 ${RATE}MB  남은 디스크 ${FREE}GB (약 ${LEFT})"
    if [ "$FREE" -lt "$STOP_FREE_GB" ]; then WHY="남은 디스크 ${FREE}GB < ${STOP_FREE_GB}GB"; say "경고: $WHY. 기록을 닫는다"; break; fi
  fi
done

# ---- 안전 종료: SIGINT → metadata.yaml 을 기다린다 → 없으면 reindex
trap '' INT TERM HUP
echo "STOPPING: $WHY" > "$STATUS"
kill -INT "$PID" 2>/dev/null
for _ in $(seq 1 15); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
if kill -0 "$PID" 2>/dev/null; then say "기록기가 15초 안에 안 끝났다. SIGTERM"; kill -TERM "$PID" 2>/dev/null; sleep 3; fi
kill -0 "$PID" 2>/dev/null && { say "강제 종료(SIGKILL)"; kill -KILL "$PID" 2>/dev/null; sleep 1; }
wait "$PID" 2>/dev/null
if [ ! -f "$BAG/metadata.yaml" ] && [ -d "$BAG" ]; then
  say "metadata.yaml 이 없다. reindex 로 복구한다"
  ros2 bag reindex "$BAG" "$STORAGE" >>"$LOG" 2>&1 || ros2 bag reindex "$BAG" -s "$STORAGE" >>"$LOG" 2>&1 || say "경고: reindex 실패. $LOG 를 본다"
fi
echo "STOPPED: $WHY" > "$STATUS"
echo "끝: $(date '+%Y-%m-%d %H:%M:%S %z')  ($WHY)" >> "$SESSION/notes.txt"

# ---- 마무리 점검
RC=0
if [ -f "$BAG/metadata.yaml" ]; then
  ros2 bag info "$BAG" > "$SESSION/info.txt" 2>&1
  if [ "$QUIET" = 1 ]; then python3 "$HERE/bag_check.py" "$SESSION" >> "$LOG" 2>&1 || RC=1
  else python3 "$HERE/bag_check.py" "$SESSION" | tee -a "$LOG"; [ "${PIPESTATUS[0]}" = 0 ] || RC=1; fi
else
  say "경고: 기록이 온전하지 않다(metadata.yaml 없음)"; RC=1
fi
if [ -n "${MAICON_CODE:-}${MAICON_MISSION_CODE:-}" ]; then
  python3 "$HERE/scan_secret.py" "$SESSION" >> "$LOG" 2>&1 || { say "경고: 임무코드가 기록에 들어 있다. DO_NOT_SHARE 표시를 남겼다"; RC=1; }
fi
say "기록 끝: $SESSION ($WHY)"
exit "$RC"
