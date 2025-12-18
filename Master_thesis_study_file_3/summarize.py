#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
13/14/15/16 그룹 CSV들을 자동으로 읽어,
- 드롭 윈도우에서 '처리 바이트 평균'(service_bytes_avg_per_slot)의 '평균의 평균'
- 드롭 윈도우에서 'backlog 바이트 중앙값'(backlog_bytes_median_per_slot)의 '평균의 평균'
을 계산해 화면에 출력하고, 요약 CSV도 파일로 저장합니다.

입력 CSV는 show_elbow_illine.py --out-csv 로 생성한 파일이어야 하며,
최소한 다음 컬럼이 존재해야 합니다:
  - service_bytes_avg_per_slot      (bytes/0.1s)
  - backlog_bytes_median_per_slot   (bytes)

사용법: 인자 없이 그냥 실행
"""

from __future__ import annotations
from pathlib import Path
import pandas as pd
import numpy as np
import re
import sys

# ----- 기본 입력/출력 파일 -----				# 파일 이름 설정하는 부분
CSV_FILES = [
    "service_and_backlog_window_avg_13.csv",
    "service_and_backlog_window_avg_14.csv",
    "service_and_backlog_window_avg_15.csv",
    "service_and_backlog_window_avg_16.csv",
]
OUTPUT_CSV = "group_means_summary.csv"

# ----- 설정 -----
SVC_COL = "service_bytes_avg_per_slot"        # bytes/0.1s
BKG_COL = "backlog_bytes_median_per_slot"     # bytes
RE_GROUP = re.compile(r"[_/-](\d+)\.csv$", re.IGNORECASE)


def infer_group_id(path: Path) -> str:
    """파일명 끝의 _13.csv / _14.csv / ... 형태에서 그룹 번호 추출(없으면 stem)."""
    m = RE_GROUP.search(path.as_posix())
    return m.group(1) if m else path.stem


def summarize_one(csv_path: Path,
                  svc_col: str = SVC_COL,
                  bkg_col: str = BKG_COL) -> dict:
    """
    한 CSV에서 유효 행만 골라 두 컬럼의 '평균의 평균'을 계산해 반환.
    - 유효 행: slots > 0 (있으면), 그리고 두 컬럼이 숫자인 행
    - service_bytes_avg_per_slot  : bytes/0.1s 값들의 산술평균
    - backlog_bytes_median_per_slot : bytes 값들의 산술평균
    """
    group = infer_group_id(csv_path)
    if not csv_path.exists():
        return {"group": group, "error": f"file_not_found: {csv_path}"}

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        return {"group": group, "error": f"read_error: {e}"}

    missing = [c for c in [svc_col, bkg_col] if c not in df.columns]
    if missing:
        return {"group": group, "error": f"missing_columns: {missing}"}

    # 유효 행 필터링
    if "slots" in df.columns:
        df = df[df["slots"].fillna(0) > 0]
    df[svc_col] = pd.to_numeric(df[svc_col], errors="coerce")
    df[bkg_col] = pd.to_numeric(df[bkg_col], errors="coerce")
    df = df[df[[svc_col, bkg_col]].notna().all(axis=1)]

    if df.empty:
        return {"group": group, "error": "no_valid_rows"}

    svc_mean = float(df[svc_col].mean())   # 처리 바이트 평균의 평균 (bytes/0.1s)
    bkg_mean = float(df[bkg_col].mean())   # backlog 중앙값의 평균 (bytes)

    return {
        "group": group,
        "n_files": int(df.shape[0]),
        "service_bytes_avg_per_slot__mean_of_means": svc_mean,
        "backlog_bytes_median_per_slot__mean": bkg_mean,
    }


def main():
    rows = []
    for p in CSV_FILES:
        rows.append(summarize_one(Path(p)))

    summary = pd.DataFrame(rows)

    # 경고 출력
    if "error" in summary.columns and summary["error"].notna().any():
        sys.stderr.write("[WARN] issues found:\n")
        sys.stderr.write(summary[summary["error"].notna()][["group","error"]].to_string(index=False) + "\n")

    # 정상 그룹만 사용해 전체 평균(= 그룹 평균의 평균) 계산
    ok_mask = summary["error"].isna() if "error" in summary.columns else np.ones(len(summary), dtype=bool)
    ok = summary[ok_mask].copy()
    if ok.empty:
        print("(no valid groups)")
        return

    overall = {
        "group": "OVERALL_MEAN",
        "n_files": int(ok["n_files"].sum()) if "n_files" in ok.columns else np.nan,
        "service_bytes_avg_per_slot__mean_of_means": float(ok["service_bytes_avg_per_slot__mean_of_means"].mean()),
        "backlog_bytes_median_per_slot__mean": float(ok["backlog_bytes_median_per_slot__mean"].mean()),
    }

    # 화면 출력
    pd.set_option("display.width", 140)
    pd.set_option("display.precision", 2)

    print("\n=== 그룹별 요약 (드롭 윈도우) ===")
    keep = ["group","n_files","service_bytes_avg_per_slot__mean_of_means","backlog_bytes_median_per_slot__mean","error"]
    keep = [c for c in keep if c in summary.columns]
    print(summary[keep].to_string(index=False))

    print("\n=== 전체 평균(= 그룹별 평균의 평균) ===")
    print(pd.DataFrame([overall]).to_string(index=False))

    # CSV 저장
    out_df = pd.concat([summary[keep], pd.DataFrame([overall])], ignore_index=True)
    out_path = Path(OUTPUT_CSV).resolve()
    out_df.to_csv(out_path, index=False)
    print(f"\n[INFO] saved summary CSV to: {out_path}")


if __name__ == "__main__":
    main()
