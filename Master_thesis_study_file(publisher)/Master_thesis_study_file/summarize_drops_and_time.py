
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
summarize_drops_and_time.py

디렉토리 안의
  - mac_aqm_tid0_ac2_*_2025_N.csv
  - Time_N.csv
쌍을 찾아 N별 드랍 수 / 전송 수 / 드랍율과
시간 파일에서 START~END 소요시간(초)을 계산해 요약 CSV를 생성합니다.

출력:
  - drops_summary_per_run.csv : N별 결과
  - drops_summary_overall.csv : 전체 집계/평균

사용법:
  python3 summarize_drops_and_time.py [TARGET_DIR]

인자가 없으면 현재 작업 디렉토리를 사용합니다.
"""

import os, re, glob, sys
from typing import Dict, Tuple, Optional, List
import pandas as pd

# -----------------------
# 유틸: CSV 읽기 (인코딩/구분자 베스트에포트)
# -----------------------
def read_csv_best_effort(path: str) -> pd.DataFrame:
    last_err = None
    for enc in ("utf-8", "utf-8-sig", "cp949", "latin1"):
        for sep in (",", ";", "\t", "|"):
            try:
                return pd.read_csv(path, encoding=enc, sep=sep, engine="python")
            except Exception as e:
                last_err = e
                continue
    raise RuntimeError(f"CSV 읽기 실패: {path} ({last_err})")

# -----------------------
# time_iso에서 시작/끝 추출
#  - time_iso 컬럼에서 파싱 가능한 시간값을 모두 모아
#    min을 START, max를 END로 간주 (잡음에 강함)
# -----------------------
def extract_start_end_from_time_csv(path: str) -> Tuple[Optional[pd.Timestamp], Optional[pd.Timestamp]]:
    df = read_csv_best_effort(path)
    if 'time_iso' not in df.columns:
        # 컬럼명이 다를 경우를 대비한 느슨한 매칭
        candidates = [c for c in df.columns if str(c).strip().lower().replace(" ", "") == "time_iso"]
        if candidates:
            df.rename(columns={candidates[0]: "time_iso"}, inplace=True)
        else:
            return (None, None)

    # 파싱 가능한 값만 추리기
    times = pd.to_datetime(df['time_iso'], errors='coerce', utc=True, infer_datetime_format=True)
    times = times.dropna()
    if len(times) == 0:
        return (None, None)

    return (times.min(), times.max())

# -----------------------
# mac 파일에서 드랍/전송 집계
# -----------------------
def aggregate_mac_file(path: str) -> Tuple[Optional[float], Optional[float]]:
    df = read_csv_best_effort(path)
    # 컬럼명 느슨한 매칭
    def find_col(df, wanted: str) -> Optional[str]:
        if wanted in df.columns: 
            return wanted
        lw = wanted.lower()
        for c in df.columns:
            if str(c).strip().lower() == lw:
                return c
        return None

    drops_col = find_col(df, "ddrops_delta")
    tx_col    = find_col(df, "dtx_pkts")

    if drops_col is None or tx_col is None:
        # 흔한 오탈자/변형도 시도
        alt_map = {
            "ddrops_delta": ["drops_delta", "d_drops_delta", "ddrop_delta", "drops"],
            "dtx_pkts": ["tx_pkts", "dtx_packets", "tx_packets"],
        }
        if drops_col is None:
            for alt in alt_map["ddrops_delta"]:
                if alt in df.columns:
                    drops_col = alt; break
        if tx_col is None:
            for alt in alt_map["dtx_pkts"]:
                if alt in df.columns:
                    tx_col = alt; break

    if drops_col is None or tx_col is None:
        return (None, None)

    drops = float(pd.to_numeric(df[drops_col], errors='coerce').fillna(0).sum())
    tx    = float(pd.to_numeric(df[tx_col],    errors='coerce').fillna(0).sum())
    return (drops, tx)

# -----------------------
# 파일 패턴 스캔 및 N 매칭
# -----------------------
def collect_files(base_dir: str):
    # mac 파일은 MAC 주소가 포함되어 변형될 수 있으므로 느슨한 패턴 사용
    mac_paths = glob.glob(os.path.join(base_dir, "mac_aqm_tid0_ac2_*_2025_*.csv"))
    time_paths = glob.glob(os.path.join(base_dir, "Time_*.csv"))

    # N 추출용 정규식 (마지막 _숫자.csv)
    rex = re.compile(r"_(\d+)\.csv$")

    def index_by_N(paths: List[str]) -> Dict[int, str]:
        out = {}
        for p in paths:
            m = rex.search(p)
            if m:
                out[int(m.group(1))] = p
        return out

    mac_by_n = index_by_N(mac_paths)
    time_by_n = index_by_N(time_paths)
    all_ns = sorted(set(mac_by_n.keys()) | set(time_by_n.keys()))
    return all_ns, mac_by_n, time_by_n

# -----------------------
# 메인
# -----------------------
def main():
    base_dir = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()

    ns, mac_by_n, time_by_n = collect_files(base_dir)
    if not ns:
        print("처리할 N 파일을 찾지 못했습니다.")
        return 2

    rows = []
    total_drops = 0.0
    total_tx = 0.0

    for n in ns:
        mac_p = mac_by_n.get(n)
        time_p = time_by_n.get(n)

        drops, tx = (None, None)
        if mac_p:
            drops, tx = aggregate_mac_file(mac_p)

        start_ts = end_ts = None
        duration_sec = None
        if time_p:
            start_ts, end_ts = extract_start_end_from_time_csv(time_p)
            if start_ts is not None and end_ts is not None:
                duration_sec = float((end_ts - start_ts).total_seconds())

        drop_rate = None
        if drops is not None and tx is not None and (drops + tx) > 0:
            drop_rate = drops / (drops + tx)

        # 집계
        if drops is not None:
            total_drops += drops
        if tx is not None:
            total_tx += tx

        rows.append({
            "N": n,
            "mac_csv": mac_p or "",
            "time_csv": time_p or "",
            "drops": drops,
            "tx_pkts": tx,
            "drop_rate": drop_rate,
            "start_iso": start_ts.isoformat() if start_ts is not None else None,
            "end_iso": end_ts.isoformat() if end_ts is not None else None,
            "duration_sec": duration_sec,
        })

    per_df = pd.DataFrame(rows).sort_values("N").reset_index(drop=True)

    # 전체 집계/평균
    overall = {
        "total_runs": int(len(per_df)),
        "sum_drops": float(pd.to_numeric(per_df["drops"], errors="coerce").fillna(0).sum()),
        "sum_tx_pkts": float(pd.to_numeric(per_df["tx_pkts"], errors="coerce").fillna(0).sum()),
        # 가중 평균(전체 관점 드랍율)
        "overall_drop_rate_weighted": None,
        # 단순 평균(각 N별 드랍율 평균)
        "avg_drop_rate_unweighted": None,
        "avg_duration_sec": None,
        "avg_drops_per_run": None,
        "avg_tx_pkts_per_run": None,
    }

    denom = overall["sum_drops"] + overall["sum_tx_pkts"]
    if denom > 0:
        overall["overall_drop_rate_weighted"] = overall["sum_drops"] / denom

    if per_df["drop_rate"].notna().any():
        overall["avg_drop_rate_unweighted"] = float(pd.to_numeric(per_df["drop_rate"], errors="coerce").dropna().mean())

    if per_df["duration_sec"].notna().any():
        overall["avg_duration_sec"] = float(pd.to_numeric(per_df["duration_sec"], errors="coerce").dropna().mean())

    if len(per_df) > 0:
        overall["avg_drops_per_run"] = float(pd.to_numeric(per_df["drops"], errors="coerce").fillna(0).mean())
        overall["avg_tx_pkts_per_run"] = float(pd.to_numeric(per_df["tx_pkts"], errors="coerce").fillna(0).mean())

    out_per = os.path.join(base_dir, "drops_summary_per_run.csv")
    out_overall = os.path.join(base_dir, "drops_summary_overall.csv")
    per_df.to_csv(out_per, index=False, encoding="utf-8")
    pd.DataFrame([overall]).to_csv(out_overall, index=False, encoding="utf-8")

    print(f"[OK] {len(per_df)}개 N 처리")
    print(f" - per-run:   {out_per}")
    print(f" - overall:   {out_overall}")

if __name__ == "__main__":
    raise SystemExit(main())
