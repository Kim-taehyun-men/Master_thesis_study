#!/usr/bin/env bash
# watch_2x28.sh
# CSV를 폴링하여 "2x28" 조건(2필드 유효행 >= 28) 만족 시 TX 로그를 변환
# - 변환 명령/파일은 기존과 동일: python3 CONVERT_SCRIPT TX_LOG_PATH --outfile <...>
# - Ubuntu / bash

set -Eeuo pipefail

# ===== 환경 변수(수정 X) =====
WORK_DIR="${WORK_DIR:-$PWD}"
CSV_BASENAME="${CSV_BASENAME:-fastdds_discovery_hb.csv}"

CONVERT_SCRIPT="${CONVERT_SCRIPT:-convert_ts.py}"
TX_LOG_PATH="${TX_LOG_PATH:-/tmp/fastdds_udp_tx.log}"
TX_OUT_BASENAME="${TX_OUT_BASENAME:-fastdds_udp_tx_hb_data}"

TX_CHECK_INTERVAL="${TX_CHECK_INTERVAL:-0.5}"  # 폴링 주기(초)#!/usr/bin/env bash
# watch_2x28.sh
# CSV를 폴링하여 "2x28"(NF==2 유효행 >= 28) 만족 시 TX 로그를 변환
# 시작/종료(또는 중단) 시각을 "스크립트를 시작한 디렉토리"의 Time.csv 에 기록
# - 변환 명령/파일: python3 CONVERT_SCRIPT TX_LOG_PATH --outfile <...>
# - Ubuntu / bash

set -Eeuo pipefail

################### USER SETTINGS ###################
WORK_DIR="${WORK_DIR:-$PWD}"
CSV_BASENAME="${CSV_BASENAME:-fastdds_discovery_hb.csv}"

CONVERT_SCRIPT="${CONVERT_SCRIPT:-convert_ts.py}"
TX_LOG_PATH="${TX_LOG_PATH:-/tmp/fastdds_udp_tx.log}"
TX_OUT_BASENAME="${TX_OUT_BASENAME:-fastdds_udp_tx_hb_data}"

TX_CHECK_INTERVAL="${TX_CHECK_INTERVAL:-0.5}"   # 체크 주기(초)
ROWS_N="${ROWS_N:-28}"                           # 2xK 의 K (노드 수 만큼 적용) 14vs14vs14vs14 상황이면 28 // 10vs10vs10vs10 상황이면 20   (실험에 따라 변경할 변수)
ROUND="${ROUND:-1}"                             # 라운드 번호(로그 표기용)
TX_MARK="${TX_MARK:-${WORK_DIR}/.tx_round_${ROUND}.done}"
################### END USER SETTINGS ###################

# ===== 시작 디렉토리/시간 기록 파일 =====
START_DIR="$(pwd -P)"                           # 스크립트 시작 시점의 디렉토리
TIME_CSV="${TIME_CSV:-${START_DIR}/Time.csv}"   # 시작 디렉토리에 기록
mkdir -p "$WORK_DIR"
touch "$TIME_CSV" 2>/dev/null || true
if ! grep -q '^round,event,time_iso,time_epoch' "$TIME_CSV" 2>/dev/null; then
  echo "round,event,time_iso,time_epoch" >> "$TIME_CSV"
fi

now_iso()   { date '+%Y-%m-%d %H:%M:%S%z'; }
now_epoch() { date +%s; }

START_ISO="$(now_iso)"
START_EPOCH="$(now_epoch)"
echo "${ROUND},START,${START_ISO},${START_EPOCH}" >> "$TIME_CSV"

# 중단 시(CTRL-C 등)에도 종료 기록 남기기
abort_log_and_exit() {
  local END_ISO END_EPOCH
  END_ISO="$(now_iso)"
  END_EPOCH="$(now_epoch)"
  echo "${ROUND},ABORT,${END_ISO},${END_EPOCH}" >> "$TIME_CSV"
  exit 130
}
trap 'abort_log_and_exit' INT TERM

# ===== 2xN 검사(주석/빈줄 제외, NF==2 카운트가 N 이상이면 true) =====
is_csv_2xN() {
  local f="$1"
  local need="${2:-$ROWS_N}"
  [[ -f "$f" ]] || return 1
  awk -v need="$need" '
    BEGIN { FS = "[[:space:],;|]+"; c = 0 }
    {
      sub(/\r$/, "", $0)                 # CR 제거(Windows 호환)
      if ($0 ~ /^[[:space:]]*#/) next    # 주석(#) 무시
      if (NF == 0) next                  # 빈 줄 무시
      if (NF == 2) { c++ }               # 2필드 행 카운트
      if (c >= need) { exit 0 }          # 조기 성공 종료
    }
    END { exit c>=need ? 0 : 1 }
  ' "$f" 2>/dev/null
}

CSV_LIVE="$WORK_DIR/$CSV_BASENAME"
echo "[INFO][round=$ROUND] watching '$CSV_LIVE' for 2x${ROWS_N}; tx='$TX_LOG_PATH'"

while true; do
  if [[ -f "$CSV_LIVE" ]]; then
    if is_csv_2xN "$CSV_LIVE" "$ROWS_N"; then
      # TX 로그가 늦게 생길 수 있으므로 잠깐 대기하며 존재 확인(최대 ~5초)
      for ((k=0; k<20; k++)); do
        [[ -f "$TX_LOG_PATH" ]] && break
        sleep 0.25
      done

      tx_out="${WORK_DIR}/${TX_OUT_BASENAME}_${ROUND}"
      echo "[${ROUND}][watch] 2x${ROWS_N} 감지 → 변환 실행: python3 ${CONVERT_SCRIPT} ${TX_LOG_PATH} --outfile ${tx_out}"

      if python3 "$CONVERT_SCRIPT" "$TX_LOG_PATH" --outfile "$tx_out"; then
        : > "$TX_MARK"   # 마커 생성(재시도 방지)
        echo "[${ROUND}][watch] 변환 완료(마커 생성) → $tx_out"
      else
        echo "[${ROUND}][watch] 경고: 변환 실패(종료 후 재시도 필요할 수 있음)"
      fi

      # 종료 시간 기록
      END_ISO="$(now_iso)"
      END_EPOCH="$(now_epoch)"
      echo "${ROUND},END,${END_ISO},${END_EPOCH}" >> "$TIME_CSV"
      exit 0
    fi
    printf '[%s][watch] 감시중... CSV는 존재, 아직 2x%s 아님 (%s)\n' "$ROUND" "$ROWS_N" "$(date +%T)"
  else
    printf '[%s][watch] 감시중... CSV 없음 (%s)\n' "$ROUND" "$(date +%T)"
  fi
  sleep "$TX_CHECK_INTERVAL"
done

ROWS_N="${ROWS_N:-28}"                         # 2xN 의 N (기본 28)
ROUND="${ROUND:-1}"                             # 로그 표기를 위한 라운드 번호(기본 1)
TX_MARK="${TX_MARK:-${WORK_DIR}/.tx_round_${ROUND}.done}"

mkdir -p "$WORK_DIR"

# 2xN 형태(주석/빈 줄 제외, NF==2 인 줄이 N개 이상) 검사
is_csv_2xN() {
  local f="$1"
  local need="${2:-$ROWS_N}"
  [[ -f "$f" ]] || return 1
  awk -v need="$need" '
    BEGIN { FS = "[[:space:],;|]+"; c = 0 }
    {
      sub(/\r$/, "", $0)                 # CR 제거(Windows 호환)
      if ($0 ~ /^[[:space:]]*#/) next    # 주석(#) 무시
      if (NF == 0) next                  # 빈 줄 무시
      if (NF == 2) { c++ }               # 2필드 행 카운트
      if (c >= need) { exit 0 }          # 조기 성공 종료
    }
    END { exit c>=need ? 0 : 1 }
  ' "$f" 2>/dev/null
}

CSV_LIVE="$WORK_DIR/$CSV_BASENAME"
echo "[INFO][round=$ROUND] watching '$CSV_LIVE' for 2x${ROWS_N}; tx='$TX_LOG_PATH'"

while true; do
  if [[ -f "$CSV_LIVE" ]]; then
    if is_csv_2xN "$CSV_LIVE" "$ROWS_N"; then
      # TX 로그 파일이 늦게 생길 수 있으므로 잠깐 대기하며 존재 확인(최대 ~5초)
      for ((k=0; k<20; k++)); do
        [[ -f "$TX_LOG_PATH" ]] && break
        sleep 0.25
      done
      tx_out="${WORK_DIR}/${TX_OUT_BASENAME}_${ROUND}"
      echo "[${ROUND}][watch] 2x${ROWS_N} 감지 → 변환 실행: python3 ${CONVERT_SCRIPT} ${TX_LOG_PATH} --outfile ${tx_out}"
      if python3 "$CONVERT_SCRIPT" "$TX_LOG_PATH" --outfile "$tx_out"; then
        : > "$TX_MARK"   # 마커 생성(재시도 방지)
        echo "[${ROUND}][watch] 변환 완료(마커 생성) → $tx_out"
      else
        echo "[${ROUND}][watch] 경고: 변환 실패(종료 후 재시도 필요할 수 있음)"
      fi
      exit 0
    fi
    printf '[%s][watch] 감시중... CSV는 존재, 아직 2x%s 아님 (%s)\n' "$ROUND" "$ROWS_N" "$(date +%T)"
  else
    printf '[%s][watch] 감시중... CSV 없음 (%s)\n' "$ROUND" "$(date +%T)"
  fi
  sleep "$TX_CHECK_INTERVAL"
done
