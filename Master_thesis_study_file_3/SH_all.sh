#!/usr/bin/env bash
set -euo pipefail

# sudo로 스크립트를 돌린 경우, 실제 로그인 사용자
WRITER_USER="${SUDO_USER:-$USER}"

############################################
# 사용자 설정(환경변수로 오버라이드 가능)
############################################
#IFS=' ' read -r -a STAGGERS <<< "${STAGGERS:-0 0.2 0.3 0.5 0.7 1 1.3 1.5 2}"   # 예: export STAGGERS="0 0.2 0.5 1"
#IFS=' ' read -r -a STAGGERS <<< "${STAGGERS:-0 0.5 1 1.5 2 3 4}"   # 예: export STAGGERS="0 0.2 0.5 1"
IFS=' ' read -r -a STAGGERS <<< "${STAGGERS:-0}"   # 예: export STAGGERS="0 0.2 0.5 1"
ROUNDS=${ROUNDS:-1}					# 측정할 라운스 설정 (변경 해야할 변수)
REST_BETWEEN_CYCLES=${REST_BETWEEN_CYCLES:-15}
REST_BETWEEN_ROUNDS=${REST_BETWEEN_ROUNDS:-15}
NODE_RUNTIME=${NODE_RUNTIME:-50}			# 측정할 라운스  시간 설정 (변경 해야할 변수)
GLOBAL_WINDOW=${GLOBAL_WINDOW:-50}			# 측정할 라운스  시간 설정 (변경 해야할 변수)
STOP_SIGNAL=${STOP_SIGNAL:-SIGINT}
KILL_AFTER=${KILL_AFTER:-1s}
WAIT_UP_TIMEOUT=${WAIT_UP_TIMEOUT:-10}
##############################################
# AQM 환경
AQM_IFACE="${AQM_IFACE:-wlx94a67e6e4790}"		# 측정할 무선랜카드 이름 설정 (변경 해야할 변수)
AQM_PEER="${AQM_PEER:-70:5d:cc:38:d7:42}"		# 측정할 무선랜카드 mac ip 설정 (변경 해야할 변수)
AQM_INTERVAL="${AQM_INTERVAL:-0.1}"
AQM_PIDFILE="${AQM_PIDFILE:-/tmp/aqm_capture.pid}"
AQM_DEBUG="${AQM_DEBUG:-/tmp/_aqm_debug.log}"
############################################
# 사용자 설정(환경변수로 오버라이드 가능)
############################################


# 출력 경로(기본 /tmp — 기존 포맷 호환)
LOG_ROOT="${LOG_ROOT:-/tmp}"
RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
RUN_DIR="${RUN_DIR:-$LOG_ROOT/run_$RUN_ID}"
AQM_OUTDIR="${AQM_OUTDIR:-/tmp}"
CONV_OUTDIR="${CONV_OUTDIR:-/tmp}"

# 헤드리스 고정(gnome-terminal 안 씀)
USE_GNOME_TERMINAL="${USE_GNOME_TERMINAL:-no}"

# 변환 스크립트/입력 로그
CONVERT_TS="${CONVERT_TS:-/tmp/convert_ts.py}"
FASTDDS_LOG_IN="${FASTDDS_LOG_IN:-/tmp/fastdds_udp_tx.log}"

# ROS2 노드 목록
nodes=( "my_topic_example:publisher" )
for i in $(seq 1 12); do nodes+=( "my_topic_example_${i}:publisher_${i}" ); done 		# 노드 개수 설정  (변경 해야할 변수)

############################################
# sudo keep-alive
############################################
sudo -v || { echo "[ERR] sudo 인증 실패"; exit 1; }
( while true; do sudo -n true; sleep 60; done ) & SUDO_KEEPALIVE_PID=$!

############################################
# 유틸
############################################
ts()  { date +%T; }
log() { printf "[%s] %s\n" "$(ts)" "$*"; }
err() { printf "[%s] [ERR] %s\n" "$(ts)" "$*" >&2; }
die() { err "$*"; exit 1; }

nowf() { LC_ALL=C date +%s.%N; }
fadd() { awk -v a="$1" -v b="$2" 'BEGIN{printf "%.3f", a+b}'; }
fsub() { awk -v a="$1" -v b="$2" 'BEGIN{printf "%.3f", a-b}'; }
fmin() { awk -v a="$1" -v b="$2" 'BEGIN{printf "%.3f", (a<b?a:b)}'; }
fpos() { awk -v x="$1" 'BEGIN{if(x>0) printf "1"; else printf "0"}'; }
ceil_int() { awk -v x="$1" 'BEGIN{print (x==int(x)?x:int(x)+1)}'; }
require_cmd() { command -v "$1" >/dev/null 2>&1 || die "필수 명령 누락: $1"; }

############################################
# 사전 점검 및 디렉터리
############################################
preflight() {
  for c in awk timeout bash setsid pgrep pkill tee stat; do require_cmd "$c"; done
  [[ -d "/sys/class/net/${AQM_IFACE}" ]] || die "네트워크 인터페이스 없음: ${AQM_IFACE}"
  mkdir -p "$RUN_DIR" || true
  log "RUN_DIR: $RUN_DIR"
}
preflight

############################################
# AQM 내부 스크립트 생성 (root로 실행)
############################################
AQM_INNER="${AQM_INNER:-/tmp/_aqm_inner.sh}"
build_aqm_inner() {
  cat > "$AQM_INNER" <<'AQM_INNER_EOF'
#!/bin/sh
# _aqm_inner.sh : root로 실행 (set -u 만 사용)
set -u

: "${AQM_DEBUG:=/tmp/_aqm_debug.log}"
echo "[`date +%T`] AQM inner start" >> "$AQM_DEBUG" 2>&1

export PATH="/usr/sbin:/sbin:/usr/bin:/bin:$PATH"

# 1) debugfs
mountpoint -q /sys/kernel/debug || mount -t debugfs debugfs /sys/kernel/debug || true

# 2) 파라미터
IFACE="${IFACE:?missing IFACE}"
PEER="${PEER:-}"
INTERVAL="${INTERVAL:-1}"
AQM_PIDFILE="${AQM_PIDFILE:?missing AQM_PIDFILE}"
OUTFILE="${OUTFILE:?missing OUTFILE}"
WRITER_USER="${WRITER_USER:-}"

# 3) phy
PHY="$(basename "$(readlink -f "/sys/class/net/$IFACE/phy80211")" 2>/dev/null || true)"
[ -n "$PHY" ] || { echo "Not a mac80211 iface or phy not found: $IFACE" >&2; exit 1; }

# 4) PEER 자동 탐색
if [ -z "$PEER" ]; then
  PEER="$(iw dev "$IFACE" station dump 2>/dev/null | awk '/Station[[:space:]]/ {print $2; exit}')"
fi
[ -n "$PEER" ] || { echo "Peer MAC not found; pass PEER" >&2; exit 1; }

# 5) AQM 경로
CANDIDATES="
/sys/kernel/debug/ieee80211/$PHY/netdev:$IFACE/stations/$PEER/aqm
/sys/kernel/debug/ieee80211/$PHY/netdev:$IFACE/sta:$PEER/aqm
/sys/kernel/debug/ieee80211/$PHY/netdev:$IFACE/stations/$PEER/tid_stats
/sys/kernel/debug/ieee80211/$PHY/netdev:$IFACE/sta:$PEER/tid_stats
"
AQM=""
for p in $CANDIDATES; do [ -r "$p" ] && { AQM="$p"; break; }; done
[ -n "$AQM" ] || { echo "Readable AQM not found for $IFACE/$PEER" >&2; exit 1; }

echo "[`date +%T`] IFACE=$IFACE PEER=$PEER PHY=$PHY" >> "$AQM_DEBUG" 2>&1
echo "[`date +%T`] AQM_PATH=$AQM" >> "$AQM_DEBUG" 2>&1
head -n 3 "$AQM" >> "$AQM_DEBUG" 2>&1 || true

# 6) 출력 파일(헤더)
OUT="$OUTFILE"
mkdir -p "$(dirname "$OUT")"
( umask 000; printf "ts,dtx_bytes,dtx_pkts,bytes_1s,backlog_bytes,backlog_pkts,ddrops_delta\n" > "$OUT" )
chmod 666 "$OUT" 2>/dev/null || true
if [ -n "$WRITER_USER" ]; then
  chown "$WRITER_USER":"$WRITER_USER" "$OUT" 2>/dev/null || true
elif [ -n "${SUDO_UID:-}" ] && [ -n "${SUDO_GID:-}" ]; then
  chown "$SUDO_UID:$SUDO_GID" "$OUT" 2>/dev/null || true
fi

# 7) 세션 PGID 기록
echo "$$" > "$AQM_PIDFILE"; chmod 644 "$AQM_PIDFILE" 2>/dev/null || true

# 8) 루프(쓰기=사용자 권한으로 tee -a)
PTB=0; PTP=0; PDR=0
BB=0; BP=0; DR=0; TB=0; TP=0
DB=0; DP=0; DD=0
HAVE_BASE=0                         # ★ 기준 샘플 세팅 여부
trap 'echo "[`date +%T`] AQM inner exit" >> "$AQM_DEBUG"; exit 0' INT TERM

wr_tee() {
  if [ -n "$WRITER_USER" ]; then
    sudo -n -u "$WRITER_USER" tee -a -- "$OUT" >/dev/null 2>>"$AQM_DEBUG"
  else
    tee -a -- "$OUT" >/dev/null 2>>"$AQM_DEBUG"
  fi
}

# (선택) 부트스트랩 1줄(0값) — 보기 편하게 첫 줄 0 보장
ts0="$(LC_ALL=C date +%Y-%m-%dT%H:%M:%S.%N%z)"
printf "\"%s\",0,0,0,0,0,0\n" "$ts0" | wr_tee || true

while :; do
  ts="$(LC_ALL=C date +%Y-%m-%dT%H:%M:%S.%N%z)"

  # 커널 포맷 3종 대응: tid=0, ac=2
  line="$(awk '
      $1==0 && $2==2 {
        if (NF>=11) { print $3,$4,$6,$10,$11; exit }   # bb bp drops txB txP
        else if (NF>=9) { print $3,$4,$5,$8,$9; exit }
        else if (NF>=7) { print $3,$4,$5,$6,$7; exit }
      }
    ' "$AQM" 2>/dev/null || true)"

  if [ -n "$line" ]; then
    set -- $line
    BB="$1"; BP="$2"; DR="$3"; TB="$4"; TP="$5"
  fi

  # ★ 기준 샘플: 첫 유효 라인에서 baseline만 잡고 이 턴은 0으로/건너뜀
  if [ "$HAVE_BASE" -eq 0 ] && [ -n "$line" ]; then
    PTB="$TB"; PTP="$TP"; PDR="$DR"
    HAVE_BASE=1
    # baseline 시점에 한 줄 0을 더 남기고 다음 루프에서부터 Δ 계산
    printf "\"%s\",0,0,0,%d,%d,0\n" "$ts" "${BB:-0}" "${BP:-0}" | wr_tee || true
    sleep "${INTERVAL}" 2>/dev/null || sleep 1
    continue
  fi

  # Δ 계산
  DB=$(( TB-PTB )); [ "$DB" -lt 0 ] && DB=0
  DP=$(( TP-PTP )); [ "$DP" -lt 0 ] && DP=0
  DD=$(( DR-PDR )); [ "$DD" -lt 0 ] && DD=0
  PTB="$TB"; PTP="$TP"; PDR="$DR"

  # CSV append
  printf "\"%s\",%d,%d,%d,%d,%d,%d\n" \
         "$ts" "${DB:-0}" "${DP:-0}" "${DB:-0}" "${BB:-0}" "${BP:-0}" "${DD:-0}" | wr_tee || true

  # 3초마다 파일 상태
  if [ $(( $(date +%s) % 3 )) -eq 0 ]; then
    stat -c "[%X] ok: size=%s inode=%i path=%n" "$OUT" >> "$AQM_DEBUG" 2>&1 || true
  fi

  sleep "${INTERVAL}" 2>/dev/null || sleep 1
done
AQM_INNER_EOF
  chmod +x "$AQM_INNER"
}

############################################
# AQM 시작/정지 (timeout 래퍼 포함)
############################################
start_aqm() {
  local aqm_out="$1"
  local aqm_secs="${2:-0}"   # 남은 수집 시간(정수초), 0이면 무제한
  log "AQM 시작 → OUTFILE: $aqm_out  (limit=${aqm_secs}s)"
  build_aqm_inner
  if [[ "$aqm_secs" -gt 0 ]]; then
    sudo env IFACE="$AQM_IFACE" PEER="$AQM_PEER" OUTFILE="$aqm_out" \
             INTERVAL="$AQM_INTERVAL" AQM_PIDFILE="$AQM_PIDFILE" AQM_DEBUG="$AQM_DEBUG" \
             WRITER_USER="$WRITER_USER" \
      timeout --signal="$STOP_SIGNAL" --kill-after=2s "${aqm_secs}s" \
      setsid "$AQM_INNER" &
  else
    sudo env IFACE="$AQM_IFACE" PEER="$AQM_PEER" OUTFILE="$aqm_out" \
             INTERVAL="$AQM_INTERVAL" AQM_PIDFILE="$AQM_PIDFILE" AQM_DEBUG="$AQM_DEBUG" \
             WRITER_USER="$WRITER_USER" \
      setsid "$AQM_INNER" &
  fi
  for _ in $(seq 1 80); do [[ -s "$AQM_PIDFILE" ]] && break; sleep 0.05; done
  [[ -s "$AQM_PIDFILE" ]] || err "AQM PID 캡처 실패 — fallback(계속 진행)"
}

stop_aqm() {
  log "AQM 종료 요청($STOP_SIGNAL)"
  local pgid=""
  if [[ -s "$AQM_PIDFILE" ]]; then pgid="$(cat "$AQM_PIDFILE" 2>/dev/null || true)"; fi
  if [[ -n "${pgid:-}" ]]; then
    sudo kill -"$STOP_SIGNAL" "-$pgid" 2>/dev/null || true
    sleep 0.3
    sudo kill -SIGTERM "-$pgid" 2>/dev/null || true
    sleep 0.3
    sudo kill -SIGKILL "-$pgid" 2>/dev/null || true
  fi
  sudo pkill -f -x "/bin/sh $AQM_INNER" 2>/dev/null || true
  sudo pkill -f "_aqm_inner.sh" 2>/dev/null || true
  sudo rm -f "$AQM_PIDFILE" 2>/dev/null || true
}

############################################
# 노드 실행/종료 (헤드리스)
############################################
stop_nodes() {
  local -n _patterns=$1; local -n _terms=$2
  for pat in "${_patterns[@]}"; do pkill -f -SIGINT -- "$pat" 2>/dev/null || true; done
  sleep 1
  for pat in "${_patterns[@]}"; do pkill -f -SIGKILL -- "$pat" 2>/dev/null || true; done
  for pid in "${_terms[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
}

spawn_node() {
  local pkg="$1" exe="$2" dur="$3"
  local -n _patterns="$4"
  local -n _terms="$5"
  local pat="ros2 run $pkg $exe"
  _patterns+=( "$pat" )
  local cmd="timeout --signal=$STOP_SIGNAL --kill-after=$KILL_AFTER ${dur}s $pat"
  setsid bash -lc "$cmd" &
  _terms+=( "$!" )
}

############################################
# 한 단계 실행
############################################
run_cycle() {
  local STAGGER="$1" ROUND="$2"
  echo; echo "==================== [ROUND $ROUND / $ROUNDS | STAGGER=${STAGGER}s] ===================="

  local OUT_SUFFIX="${ROUND}_H4_P${STAGGER}(NP)"
  local AQM_OUTFILE="${AQM_OUTDIR%/}/mac_aqm_tid0_ac2_${AQM_IFACE}@${AQM_PEER}_16_${OUT_SUFFIX}.csv" 		# 출력 결과물 이름 설정 부분  mac_aqm EX)16 (변경 해야할 변수)
  local CONV_OUTFILE="${CONV_OUTDIR%/}/fastdds_discovery_udp_tx_16_${OUT_SUFFIX}.conv"				# 출력 결과물 이름 설정 부분 fastdds EX)16  (변경 해야할 변수)

  mkdir -p "$(dirname "$AQM_OUTFILE")" "$(dirname "$CONV_OUTFILE")" "$RUN_DIR" || true

  local patterns=() terms=() END_AT=""

  launch_node_wrapped() {
    IFS=":" read -r pkg exe <<< "$1"
    if [[ -z "$END_AT" ]]; then
      local start_at; start_at="$(nowf)"; END_AT="$(fadd "$start_at" "$GLOBAL_WINDOW")"
      log "첫 노드 시작 → 컷오프 T+${GLOBAL_WINDOW}s (END_AT=$END_AT)"
    fi
    local now left dur; now="$(nowf)"; left="$(fsub "$END_AT" "$now")"
    if [[ "$(fpos "$left")" == "0" ]]; then log "[SKIP] $pkg/$exe : 컷오프 경과"; return 0; fi
    dur="$(fmin "$left" "$NODE_RUNTIME")"
    spawn_node "$pkg" "$exe" "$dur" patterns terms
  }

  for i in "${!nodes[@]}"; do
    launch_node_wrapped "${nodes[$i]}"
    (( i < ${#nodes[@]} - 1 )) && sleep "$STAGGER"
  done

  # 모든 노드 '시작' 확인
  local deadline=$(( $(date +%s) + WAIT_UP_TIMEOUT ))
  while (( $(date +%s) < deadline )); do
    local up=0; for pat in "${patterns[@]}"; do pgrep -f -- "$pat" >/dev/null && ((++up)); done
    (( up == ${#patterns[@]} )) && { log "모든 노드 시작 확인(up=$up)"; break; }
    sleep 0.2
  done
  (( $(date +%s) >= deadline )) && err "일부 노드 시작 미확인 — 진행"

  # AQM 남은 수집 시간(정수 초, ceil) 계산 → timeout에 사용
  local left="$(fsub "$END_AT" "$(nowf)")"
  local left_int="$(ceil_int "$left")"; (( left_int < 1 )) && left_int=1
  start_aqm "$AQM_OUTFILE" "$left_int"

  (
    local sleep_left; sleep_left="$(fsub "$END_AT" "$(nowf)")"
    if [[ -n "$sleep_left" && "$(fpos "$sleep_left")" == "1" ]]; then sleep "$sleep_left"; fi
    log "컷오프 도달 → 모든 노드 종료($STOP_SIGNAL)"
    stop_nodes patterns terms
  ) &

  for pid in "${terms[@]}"; do wait "$pid" || true; done

  stop_aqm
  sleep 0.3

  # 변환
  if [[ -s "$FASTDDS_LOG_IN" ]]; then
    log "변환 실행: $CONVERT_TS $(basename "$FASTDDS_LOG_IN") → $CONV_OUTFILE"
    ( cd "$(dirname "$FASTDDS_LOG_IN")" && python3 "$CONVERT_TS" "$(basename "$FASTDDS_LOG_IN")" --outfile "$CONV_OUTFILE" ) || {
      err "Python 변환 실패(계속). 스크립트=$CONVERT_TS 입력=$FASTDDS_LOG_IN"
    }
    rm -f "$FASTDDS_LOG_IN" || true
  else
    err "FASTDDS 입력 로그 없음 → 변환 생략: $FASTDDS_LOG_IN"
  fi

  log "ROUND $ROUND | STAGGER=${STAGGER}s 완료"
}

############################################
# 종료 트랩
############################################
on_exit() {
  log "스크립트 종료(cleanup)"
  stop_aqm || true
  pkill -f -SIGINT -- "ros2 run " 2>/dev/null || true
  sleep 1
  pkill -f -SIGKILL -- "ros2 run " 2>/dev/null || true
  if [[ -n "${SUDO_KEEPALIVE_PID:-}" ]]; then kill "$SUDO_KEEPALIVE_PID" 2>/dev/null || true; fi
  sudo rm -f "$AQM_PIDFILE" 2>/dev/null || true
}
trap on_exit EXIT INT TERM

############################################
# 메인
############################################
for round in $(seq 1 "$ROUNDS"); do
  echo; echo "==================== [ROUND $round / $ROUNDS] ===================="
  for idx in "${!STAGGERS[@]}"; do
    run_cycle "${STAGGERS[$idx]}" "$round"
    (( idx < ${#STAGGERS[@]} - 1 )) && { log "다음 단계 전 ${REST_BETWEEN_CYCLES}s 대기"; sleep "$REST_BETWEEN_CYCLES"; }
  done
  (( round < ROUNDS )) && { log "다음 라운드($((round+1))) 전 ${REST_BETWEEN_ROUNDS}s 대기"; sleep "$REST_BETWEEN_ROUNDS"; }
done

echo; log "모든 라운드 완료"

