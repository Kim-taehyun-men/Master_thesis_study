#!/usr/bin/env bash
# ros2 publisher 반복 실행 & CSV 회차별 저장 (종료 후 리네임/삭제)
# Ubuntu / bash

set -Eeuo pipefail

# ===== 사용자 설정 (환경변수로도 덮어쓰기 가능) =====
ITERATIONS="${ITERATIONS:-1000}"           # 반복 횟수
RUN_SECS="${RUN_SECS:-40}"                # 프로세스 가동 시간(초)
WAIT_SECS="${WAIT_SECS:-10}"              # 라운드 사이 대기(초)
CSV_BASENAME="${CSV_BASENAME:-fastdds_discovery_hb.csv}"   # 원본 생성 파일명
WORK_DIR="${WORK_DIR:-$PWD}"              # CSV 생성 디렉터리
ROS_CMD="${ROS_CMD:-ros2 run my_topic_example publisher}"  # 실행할 명령

# (필요 시) ROS 환경 세팅
# source /opt/ros/humble/setup.bash
# source ~/your_ws/install/setup.bash

echo "[INFO] iterations=$ITERATIONS run_secs=$RUN_SECS wait_secs=$WAIT_SECS work_dir=$WORK_DIR"
mkdir -p "$WORK_DIR"

cleanup() {
  if [[ -n "${ROS_PGID:-}" ]] && kill -0 -"$ROS_PGID" 2>/dev/null; then
    # CTRL-C와 유사한 INT → TERM → KILL 순서로 정리
    kill -INT -"$ROS_PGID" 2>/dev/null || true
    sleep 1
    kill -TERM -"$ROS_PGID" 2>/dev/null || true
    sleep 1
    kill -KILL -"$ROS_PGID" 2>/dev/null || true
  fi
}
trap 'echo; echo "[INTERRUPT] 정리 중..."; cleanup; exit 130' INT TERM

for ((i=1; i<=ITERATIONS; i++)); do
  echo "================= [ ROUND $i / $ITERATIONS ] ================="

  # 새 세션/프로세스 그룹으로 실행하여 전체 종료가 용이하도록 함
  setsid bash -lc "$ROS_CMD" >/dev/null 2>&1 &
  ROS_PID=$!
  ROS_PGID=$ROS_PID
  echo "[$i] 프로세스 시작: pid=$ROS_PID pgid=$ROS_PGID -> '$ROS_CMD'"

  # 지정된 가동 시간만큼 실행
  sleep "$RUN_SECS" || true

  # 프로세스 그룹 종료 (INT → TERM → KILL)
  if kill -0 -"$ROS_PGID" 2>/dev/null; then
    kill -INT -"$ROS_PGID" 2>/dev/null || true
    sleep 2
    if kill -0 -"$ROS_PGID" 2>/dev/null; then
      kill -TERM -"$ROS_PGID" 2>/dev/null || true
      sleep 2
      if kill -0 -"$ROS_PGID" 2>/dev/null; then
        kill -KILL -"$ROS_PGID" 2>/dev/null || true
      fi
    fi
  fi

  # 완전히 종료될 때까지 대기
  wait "$ROS_PID" 2>/dev/null || true
  unset ROS_PGID

  # ====== 종료 완료 후에만 파일 이름 변경 & 정리 ======
  csv_path="$WORK_DIR/$CSV_BASENAME"
  target_path="$WORK_DIR/fastdds_discovery_hb_${i}.csv"

  if [[ -f "$csv_path" ]]; then
    mv -f "$csv_path" "$target_path"
    echo "[$i] 종료 후 파일 이름 변경 → $(basename "$target_path")"
  else
    echo "[$i] 경고: 종료 시점에 ${CSV_BASENAME} 가 존재하지 않습니다."
  fi

  # 혹시 남아 있으면 원본 이름 파일 제거
  if [[ -f "$csv_path" ]]; then
    rm -f "$csv_path"
    echo "[$i] 잔여 ${CSV_BASENAME} 제거"
  fi

  # 다음 라운드 전 대기
  if (( i < ITERATIONS )); then
    echo "[$i] 다음 라운드까지 ${WAIT_SECS}s 대기..."
    sleep "$WAIT_SECS"
  fi
done

echo "[DONE] 모든 라운드 완료."

