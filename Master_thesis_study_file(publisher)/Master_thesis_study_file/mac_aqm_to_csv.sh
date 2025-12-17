#!/bin/sh
# TID=0, AC=2(BE) 전송량/백로그/드롭(Δ)을 INTERVAL마다 CSV로 기록
# 'mbps' 대신 'bytes_1s' 컬럼으로 1초 처리 바이트(Δ)를 출력합니다.

set -eu

IFACE="${1:-}"
PEER="${2:-}"
OUTDIR="${3:-$PWD}"
INTERVAL="${INTERVAL:-1}"

if [ -z "$IFACE" ]; then
  echo "Usage: sudo $0 <ifname> [peer-mac] [outdir]" >&2
  exit 1
fi

# debugfs mount
mountpoint -q /sys/kernel/debug || mount -t debugfs debugfs /sys/kernel/debug

# phy 확인
PHY="$(basename "$(readlink -f /sys/class/net/$IFACE/phy80211)" 2>/dev/null || true)"
[ -n "$PHY" ] || { echo "Not a mac80211 wireless iface or phy not found: $IFACE" >&2; exit 1; }

# peer(BSSID) 자동 탐색(미지정 시)
if [ -z "$PEER" ]; then
  PEER="$(iw dev "$IFACE" station dump | awk '/Station/{print $2; exit}')"
fi
[ -n "$PEER" ] || { echo "Peer MAC not found; pass it as 2nd arg" >&2; exit 1; }

AQM="/sys/kernel/debug/ieee80211/$PHY/netdev:$IFACE/stations/$PEER/aqm"
[ -r "$AQM" ] || { echo "Cannot read $AQM (need sudo? associated?)" >&2; exit 1; }

TS="$(LC_ALL=C date +%Y%m%d_%H%M%S)"
OUT="$OUTDIR/mac_aqm_tid0_ac2_${IFACE}@${PEER}_${TS}.csv"

# CSV 헤더(7열 고정) — mbps 대신 bytes_1s
echo "ts,dtx_bytes,dtx_pkts,bytes_1s,backlog_bytes,backlog_pkts,ddrops_delta" > "$OUT"

cleanup(){ echo; echo "CSV saved: $OUT"; }
trap cleanup INT TERM EXIT

# 이전 누적값
PTB=""; PTP=""; PDR=""

while :; do
  # 로케일 고정 + 타임스탬프를 따옴표로 (CSV 열 밀림 방지)
  ts="$(LC_ALL=C date +%Y-%m-%dT%H:%M:%S.%N%z)"

  # aqm TID0/AC2 라인에서 필요한 필드만 추출
  # 포맷: tid ac backlogB(3) backlogP(4) newflows drops(6) marks overlimit collisions txB(10) txP(11)
  # 여기서는 bb(3) bp(4) dr(6) tb(10) tp(11)만 사용
  set -- $(awk '
    NF>=11 && $1==0 && $2==2 {print $3,$4,$6,$10,$11; found=1}
    END{ if(!found) print "" }
  ' "$AQM")

  if [ $# -lt 5 ]; then
    # 해당 라인이 없거나 읽기 실패 시 0으로 출력
    DB=0; DP=0; BB=0; BP=0; DD=0
  else
    BB="$1"; BP="$2"; DR="$3"; TB="$4"; TP="$5"

    # Δ 계산(기본 1초 주기)
    if [ -n "$PTB" ]; then DB=$((TB-PTB)); else DB=0; fi
    if [ -n "$PTP" ]; then DP=$((TP-PTP)); else DP=0; fi
    if [ -n "$PDR" ]; then DD=$((DR-PDR)); else DD=0; fi

    # 음수 방지(카운터 리셋 등)
    [ "$DB" -lt 0 ] && DB=0
    [ "$DP" -lt 0 ] && DP=0
    [ "$DD" -lt 0 ] && DD=0

    # 이전값 갱신
    PTB="$TB"; PTP="$TP"; PDR="$DR"
  fi

  # bytes_1s는 주기 Δ 바이트(= DB). INTERVAL≠1이면 'bytes_per_interval' 의미가 됩니다.
  BYTES_1S="$DB"

  # 숫자 형식 고정 (정수는 %d)
  printf "\"%s\",%d,%d,%d,%d,%d,%d\n" \
         "$ts" "$DB" "$DP" "$BYTES_1S" "$BB" "$BP" "$DD" >> "$OUT"

  sleep "$INTERVAL"
done

