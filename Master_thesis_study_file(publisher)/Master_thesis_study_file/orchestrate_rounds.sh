#!/usr/bin/env bash
# Orchestrates repeated "rounds" of:
#  1) sudo INTERVAL=<INTERVAL> ./mac_aqm_to_csv.sh <IFACE>
#  2) ./watch_2x28.sh
#  3) ./run_all.sh
#
# A round ends when either:
#  - timeout passes, OR
#  - watch_2x28.sh exits earlier.
#
# After each round ends, it renames files created during that round as:
#  - fastdds_discovery_hb.csv           -> fastdds_discovery_hb_<ROUND>.csv
#  - fastdds_udp_tx_hb_data* (newest)   -> fastdds_udp_tx_hb_data_<ROUND>[.ext]
#  - mac_aqm_tid0_ac2_<IFACE>@*_YYYY(MMDD_HHMMSS).csv or _YYYYMMDD_HHMMSS.csv
#      -> mac_aqm_tid0_ac2_<IFACE>@*__YYYY_<ROUND>.csv  (변하는 타임스탬프 부분 제거)
#  - Time.csv                           -> Time_<ROUND>.csv
#
# USAGE:
#   ./orchestrate_rounds.sh [--rounds N] [--iface IFACE] [--interval SEC] [--timeout SEC] [--rest SEC]
#
set -Eeuo pipefail

########## USER SETTINGS ##########
ROUNDS=${ROUNDS:-1}          			 # 라운드 수			 (실험에 따라 변경할 변수)
IFACE_DEFAULT="wlx94a67e6e4790"			 # 무선 인터페이스 이름
IFACE="${IFACE:-$IFACE_DEFAULT}"
INTERVAL="${INTERVAL:-0.1}"   			 # mac_aqm_to_csv 샘플링 간격(초)
ROUND_TIMEOUT_SEC="${ROUND_TIMEOUT_SEC:-1200}"   # 한 라운드 최대 시간(초)		(실험에 따라 변경할 변수) [Discovery 최대 시간을 위해 설정] -EX) 노드수가 적은 경우(10)이면 100초로 설정해도 괜찮을 듯함
REST_SEC="${REST_SEC:-120}"     		 # 라운드 사이 휴식(초)		(실험에 따라 변경할 변수) [네트워크 안정화를 위해 설정]      -EX) 노드수가 적은 경우(10)이면 100초로 설정해도 괜찮을 듯함  
########## END USER SETTINGS ##########

# CLI overrides
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rounds) ROUNDS="$2"; shift 2;;
    --iface) IFACE="$2"; shift 2;;
    --interval) INTERVAL="$2"; shift 2;;
    --timeout) ROUND_TIMEOUT_SEC="$2"; shift 2;;
    --rest) REST_SEC="$2"; shift 2;;
    -h|--help)
      grep '^# ' "$0" | sed 's/^# //'
      exit 0
      ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

# Record the caller directory (where the user ran the script)
CALL_DIR="$(pwd)"

# Resolve script dir and cd there so relative paths work
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

WATCH_SCRIPT="./watch_2x28.sh"
if [[ ! -x "$WATCH_SCRIPT" && -x "./watch_2X28.sh" ]]; then
  WATCH_SCRIPT="./watch_2X28.sh"
fi

RUN_ALL_SCRIPT="./run_all.sh"
MAC_AQM_SCRIPT="./mac_aqm_to_csv.sh"

require_file() {
  local f="$1"
  if [[ ! -x "$f" ]]; then
    echo "ERROR: required executable not found: $f" >&2
    exit 1
  fi
}

require_file "$RUN_ALL_SCRIPT"
require_file "$MAC_AQM_SCRIPT"
require_file "$WATCH_SCRIPT"

log() { printf '[%s] %s\n' "$(date +'%F %T')" "$*"; }

# ------- STATE DIR (writable) --------
# Default to a hidden folder in the CALL_DIR; if not writable, fallback to mktemp.
STATE_DIR_DEFAULT="$CALL_DIR/.orchestrate_state"
if mkdir -p "$STATE_DIR_DEFAULT" 2>/dev/null; then
  STATE_DIR="$STATE_DIR_DEFAULT"
else
  STATE_DIR="$(mktemp -d 2>/dev/null || mktemp -d -p "$HOME")"
fi
# Double-check writability
if [[ ! -w "$STATE_DIR" ]]; then
  echo "ERROR: No writable state dir available (tried $STATE_DIR_DEFAULT and mktemp)" >&2
  exit 1
fi

# Graceful + forceful killer for processes matching a pattern
terminate_by_pattern() {
  local pattern="$1"
  local signal_first="${2:-INT}"
  local grace="${3:-2}"

  mapfile -t pids < <(pgrep -f -- "$pattern" || true)
  if (( ${#pids[@]} == 0 )); then
    return 0
  fi

  log "Sending SIG${signal_first} to: ${pids[*]} (pattern: $pattern)"
  kill -"${signal_first}" "${pids[@]}" 2>/dev/null || true
  sleep "$grace"

  mapfile -t pids < <(pgrep -f -- "$pattern" || true)
  if (( ${#pids[@]} )); then
    log "Sending SIGTERM to remaining: ${pids[*]}"]
    kill -TERM "${pids[@]}" 2>/dev/null || true
    sleep "$grace"
  fi

  mapfile -t pids < <(pgrep -f -- "$pattern" || true)
  if (( ${#pids[@]} )); then
    log "Sending SIGKILL to stubborn: ${pids[*]}"
    kill -KILL "${pids[@]}" 2>/dev/null || true
  fi
}

# Kill an entire process tree starting from a root PID
kill_tree() {
  local root="$1"; local sig="${2:-TERM}"; local grace="${3:-2}"
  [[ -n "${root:-}" ]] || return 0
  if ! kill -0 "$root" 2>/dev/null; then return 0; fi

  # Find all descendants
  local children
  mapfile -t children < <(pgrep -P "$root" || true)
  for c in "${children[@]}"; do
    kill_tree "$c" "$sig" "$grace"
  done

  kill -"$sig" "$root" 2>/dev/null || true
  sleep "$grace"
  if kill -0 "$root" 2>/dev/null; then
    kill -KILL "$root" 2>/dev/null || true
  fi
}

# Capture gnome-terminal PIDs before/after to identify new ones
list_gterms() { pgrep -x gnome-terminal || true; }
diff_pid_sets() {
  local set1=" $1 "; local set2=" $2 "
  for pid in $set2; do
    [[ "$set1" == *" $pid "* ]] || echo "$pid"
  done
}

# Robust rename for mac_aqm files: strip variable timestamp and keep only YEAR
rename_aqm_to_year_round() {
  local src="$1" round="$2"
  local dest="" prefix="" year=""
  if [[ "$src" =~ ^(.+)_([0-9]{8})_([0-9]{6})\.csv$ ]]; then
    prefix="${BASH_REMATCH[1]}"
    year="${BASH_REMATCH[2]:0:4}"
    dest="${prefix}_${year}_${round}.csv"
  elif [[ "$src" =~ ^(.+)_([0-9]{4})\([0-9]{4}_[0-9]{6}\)\.csv$ ]]; then
    prefix="${BASH_REMATCH[1]}"
    year="${BASH_REMATCH[2]}"
    dest="${prefix}_${year}_${round}.csv"
  else
    dest="${src%.csv}_${round}.csv"
  fi

  if [[ -e "$dest" ]]; then
    local n=1
    while [[ -e "${dest%.csv}_${n}.csv" ]]; do n=$((n+1)); done
    dest="${dest%.csv}_${n}.csv"
  fi

  log "Renaming '$src' -> '$dest'"
  if ! mv -- "$src" "$dest" 2>/dev/null; then
    log "mv failed without sudo; retrying with sudo..."
    sudo mv -- "$src" "$dest"
  fi
}

# General safe move with round suffix
safemv_with_round_suffix() {
  local src="$1"
  local round="$2"
  local dest="" ext="" base=""
  [[ -e "$src" ]] || return 1
  if [[ "$src" == *.csv ]]; then base="${src%.csv}"; ext=".csv"; else base="$src"; fi
  dest="${base}_${round}${ext}"
  if [[ -e "$dest" ]]; then
    local n=1
    while [[ -e "${base}_${round}_${n}${ext}" ]]; do n=$((n+1)); done
    dest="${base}_${round}_${n}${ext}"
  fi
  log "Renaming '$src' -> '$dest'"
  if ! mv -- "$src" "$dest" 2>/dev/null; then
    log "mv failed without sudo; retrying with sudo..."
    sudo mv -- "$src" "$dest"
  fi
}

# Find newest file matching pattern created after marker
newest_since() {
  local marker="$1"; shift
  local glob="$1"; shift
  mapfile -t __files < <(ls -1t -- $glob 2>/dev/null || true)
  local f
  for f in "${__files[@]}"; do
    if [[ "$f" -nt "$marker" ]]; then
      echo "$f"
      return 0
    fi
  done
  return 1
}

cleanup() {
  log "Caught signal; shutting down all programs..."
  shutdown_all_programs || true
}
trap cleanup INT TERM

shutdown_all_programs() {
  terminate_by_pattern "$WATCH_SCRIPT" INT 1
  terminate_by_pattern 'ros2[[:space:]]+run[[:space:]]+my_topic_example(_[0-9]+)?[[:space:]]+publisher(_[0-9]+)?' INT 1
  terminate_by_pattern '/publisher(_[0-9]+)?([[:space:]]|$)' INT 1
  terminate_by_pattern 'gnome-terminal.*ros2[[:space:]]+run' INT 1
  terminate_by_pattern "$MAC_AQM_SCRIPT[[:space:]].*${IFACE}" INT 1
}

run_round() {
  local round="$1"
  log "===== ROUND $round: START ====="

  local marker="$STATE_DIR/round_${round}.start"
  : > "$marker"; touch "$marker"

  local terms_before; terms_before="$(list_gterms | tr '\n' ' ')"

  log "Starting mac_aqm_to_csv: IFACE=$IFACE, INTERVAL=$INTERVAL"
  sudo INTERVAL="$INTERVAL" "$MAC_AQM_SCRIPT" "$IFACE" &
  local pid_mac=$!

  sleep 0.7

  log "Starting watch: $WATCH_SCRIPT"
  bash "$WATCH_SCRIPT" &
  local pid_watch=$!

  sleep 0.7

  log "Starting run_all: $RUN_ALL_SCRIPT"
  bash "$RUN_ALL_SCRIPT" &
  local pid_run_all=$!

  sleep 2
  local terms_after; terms_after="$(list_gterms | tr '\n' ' ')"
  local new_terms; new_terms="$(diff_pid_sets "$terms_before" "$terms_after" | tr '\n' ' ')"
  printf '%s\n' $new_terms > "$STATE_DIR/round_${round}.gterms"

  log "Waiting for watch to finish OR timeout (${ROUND_TIMEOUT_SEC}s)..."
  (
    sleep "$ROUND_TIMEOUT_SEC"
    exit 200
  ) &
  local pid_timer=$!

  local rc
  while true; do
    if ! kill -0 "$pid_watch" 2>/dev/null; then rc=0; break; fi
    if ! kill -0 "$pid_timer" 2>/dev/null; then rc=200; break; fi
    sleep 1
  done

  if [[ "$rc" -eq 200 ]]; then
    log "TIMEOUT reached for round $round."
  else
    log "watch_2x28.sh finished early for round $round."
  fi

  kill "$pid_timer" 2>/dev/null || true

  kill_tree "$pid_watch" INT 1 || true

  if [[ -s "$STATE_DIR/round_${round}.gterms" ]]; then
    while read -r tpid; do
      [[ -n "$tpid" ]] || continue
      log "Closing gnome-terminal PID $tpid"
      kill_tree "$tpid" TERM 1 || true
    done < "$STATE_DIR/round_${round}.gterms"
  fi
  shutdown_all_programs

  kill_tree "$pid_mac" INT 1 || true

  log "Renaming round-$round artifacts..."

  if newest_since "$marker" fastdds_discovery_hb.csv >/dev/null; then
    safemv_with_round_suffix "fastdds_discovery_hb.csv" "$round" || true
  else
    [[ -e fastdds_discovery_hb.csv ]] && safemv_with_round_suffix "fastdds_discovery_hb.csv" "$round" || true
  fi

  local udp_tx_file=""
  udp_tx_file="$(newest_since "$marker" fastdds_udp_tx_hb_data* || true)"
  if [[ -n "${udp_tx_file:-}" ]]; then
    local ext=""; [[ "$udp_tx_file" == *.csv ]] && ext=".csv"
    local dest="fastdds_udp_tx_hb_data_${round}${ext}"
    if [[ -e "$dest" ]]; then
      local n=1; while [[ -e "fastdds_udp_tx_hb_data_${round}_${n}${ext}" ]]; do n=$((n+1)); done
      dest="fastdds_udp_tx_hb_data_${round}_${n}${ext}"
    fi
    log "Renaming '$udp_tx_file' -> '$dest'"
    if ! mv -- "$udp_tx_file" "$dest" 2>/dev/null; then
      log "mv failed without sudo; retrying with sudo..."
      sudo mv -- "$udp_tx_file" "$dest"
    fi
  else
    for candidate in fastdds_udp_tx_hb_data fastdds_udp_tx_hb_data_1; do
      if [[ -e "$candidate" || -e "${candidate}.csv" ]]; then
        [[ -e "$candidate" ]] && safemv_with_round_suffix "$candidate" "$round" || true
        [[ -e "${candidate}.csv" ]] && safemv_with_round_suffix "${candidate}.csv" "$round" || true
        break
      fi
    done
  fi

  local aqm_file=""
  aqm_file="$(newest_since "$marker" "mac_aqm_tid0_ac2_${IFACE}@*.csv" || true)"
  if [[ -n "${aqm_file:-}" ]]; then
    rename_aqm_to_year_round "$aqm_file" "$round" || true
  else
    aqm_file="$(newest_since "$marker" "mac_aqm_tid0_ac2_*@*.csv" || true)"
    [[ -n "${aqm_file:-}" ]] && rename_aqm_to_year_round "$aqm_file" "$round" || true
  fi

  if newest_since "$marker" Time.csv >/dev/null; then
    safemv_with_round_suffix "Time.csv" "$round" || true
  elif [[ -e "Time.csv" ]]; then
    safemv_with_round_suffix "Time.csv" "$round" || true
  fi

  if [[ -e "fastdds_udp_tx.log" ]]; then
    log "Deleting fastdds_udp_tx.log"
    if ! rm -f -- "fastdds_udp_tx.log" 2>/dev/null; then
      log "rm failed without sudo; retrying with sudo..."
      sudo rm -f -- "fastdds_udp_tx.log" || true
    fi
  fi

  rm -f -- "$marker" "$STATE_DIR/round_${round}.gterms"

  log "===== ROUND $round: END ====="
  echo
}

log "State dir: $STATE_DIR"
log "Starting orchestrator with ROUNDS=$ROUNDS, IFACE=$IFACE, INTERVAL=$INTERVAL, TIMEOUT=${ROUND_TIMEOUT_SEC}s, REST=${REST_SEC}s"
for (( r=1; r<=ROUNDS; r++ )); do
  run_round "$r"
  if (( r < ROUNDS )); then
    log "Resting ${REST_SEC}s before next round..."
    sleep "$REST_SEC"
  fi
done
log "All rounds completed."
