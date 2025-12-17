#!/usr/bin/env bash
set -euo pipefail

# 첫 번째 노드 실행
gnome-terminal -- bash -lc "ros2 run my_topic_example publisher; exec bash"

# 나머지 9개 노드 실행 (X초 간격으로 실행 문구)
for i in $(seq 1 9); do				# publisher 노드 개수 설정 [9로 설정 시 총 10개의 노드 실행 됨]     (실험에 따라 변경할 변수)
  pkg="my_topic_example_${i}"
  exe="publisher_${i}"

  #sleep 8  					# 8초의 실행 주기 설정 // 없을시 실행주기 존재하지 않은 경우(Normal) (실험에 따라 변경할 변수)
  gnome-terminal -- bash -lc "ros2 run ${pkg} ${exe}; exec bash"
done
