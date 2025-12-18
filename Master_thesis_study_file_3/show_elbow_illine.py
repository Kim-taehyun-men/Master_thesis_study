#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
드롭 윈도우(에피소드 ±neighbors 합집합)에서 '처리 vs 유입'의
- 평균 패킷 수 (pkts/0.1s)
- 평균 총 바이트 (B/0.1s)
- 평균 패킷 크기 (bytes/pkt ; 총합/총합 방식)
을 계산하고, 파일별 1행 + 전체 요약(mean/median/std/min/max)을 출력.
CSV 저장 옵션(--out-csv, --out-cols)으로 파일별 결과를 저장할 수 있음.
"""

from __future__ import annotations
import argparse
from pathlib import Path
from typing import List, Optional, Dict, Any
import numpy as np
import pandas as pd
import sys, math

SLOT_SEC = 0.1

# ---------- load ----------
def load_df(path: Path) -> pd.DataFrame:
    if path.suffix == ".gz":
        df = pd.read_csv(path, compression="gzip")
    else:
        try:
            df = pd.read_csv(path)
        except FileNotFoundError:
            alt = Path(str(path) + ".csv")
            if alt.exists(): df = pd.read_csv(alt)
            else: raise
    need = ["dtx_bytes","dtx_pkts","backlog_bytes","ddrops_delta"]
    miss = [c for c in need if c not in df.columns]
    if miss: raise ValueError(f"{path.name}: required columns missing: {miss}")

    # Δbacklog
    df["backlog_bytes_prev"] = df["backlog_bytes"].shift(1).fillna(df["backlog_bytes"])
    df["backlog_delta_bytes"] = df["backlog_bytes"] - df["backlog_bytes_prev"]

    if "backlog_pkts" in df.columns:
        df["backlog_pkts_prev"] = df["backlog_pkts"].shift(1).fillna(df["backlog_pkts"])
        df["backlog_delta_pkts"] = df["backlog_pkts"] - df["backlog_pkts_prev"]
        with np.errstate(divide="ignore", invalid="ignore"):
            df["in_size_candidate"] = np.where(df["backlog_delta_pkts"]>0,
                                               df["backlog_delta_bytes"]/df["backlog_delta_pkts"],
                                               np.nan)
            df["in_size_candidate"] = df["in_size_candidate"].ffill().bfill()
    else:
        df["backlog_delta_pkts"] = np.nan
        df["in_size_candidate"] = np.nan

    # drop bytes (가능 시)
    df["drop_bytes_est"] = np.where(np.isfinite(df["in_size_candidate"]),
                                    df["ddrops_delta"]*df["in_size_candidate"], 0.0)

    # arrival bytes
    dB_pos = np.maximum(df["backlog_delta_bytes"], 0.0)
    df["arrival_bytes_lb"]  = df["dtx_bytes"] + dB_pos
    df["arrival_bytes_est"] = df["arrival_bytes_lb"] + df["drop_bytes_est"]

    # arrival pkts (backlog_pkts 없으면 정확 계산 불가)
    if np.isfinite(df["backlog_delta_pkts"]).any():
        dB_pkts_pos = np.maximum(df["backlog_delta_pkts"], 0.0).fillna(0.0)
        df["arrival_pkts_est"] = df["dtx_pkts"] + dB_pkts_pos + df["ddrops_delta"]
    else:
        df["arrival_pkts_est"] = np.nan  # bytes 위주로 보세요

    return df

# ---------- select drop union window ----------
def drop_union_mask(df: pd.DataFrame, neighbors: int=1) -> pd.Series:
    m = (df["ddrops_delta"]>0).to_numpy()
    if neighbors>0:
        for k in range(1, neighbors+1):
            m |= np.roll(m,  k)
            m |= np.roll(m, -k)
        # 가장자리 롤오버 방지
        m[:neighbors] |= (df["ddrops_delta"].iloc[:neighbors].to_numpy()>0)
        m[-neighbors:] |= (df["ddrops_delta"].iloc[-neighbors:].to_numpy()>0)
    return pd.Series(m, index=df.index)

# ---------- per-file metrics ----------
def per_file_window_averages(df: pd.DataFrame, neighbors: int=1) -> Dict[str, Any]:
    mask = drop_union_mask(df, neighbors=neighbors)
    sub = df[mask]
    if sub.empty:
        return {"slots": 0}

    N = float(len(sub))

    # Service totals/averages
    S_bytes_total = float(sub["dtx_bytes"].sum())
    S_pkts_total  = float(sub["dtx_pkts"].sum())
    S_bytes_avg   = S_bytes_total / N
    S_pkts_avg    = S_pkts_total  / N
    S_size        = (S_bytes_total / S_pkts_total) if S_pkts_total>0 else np.nan

    # Arrival (EST) totals/averages
    A_bytes_est_total = float(sub["arrival_bytes_est"].sum())
    A_bytes_est_avg   = A_bytes_est_total / N

    if sub["arrival_pkts_est"].notna().any():
        A_pkts_est_total = float(sub["arrival_pkts_est"].sum())
        A_pkts_est_avg   = A_pkts_est_total / N
        A_size_est       = (A_bytes_est_total / A_pkts_est_total) if A_pkts_est_total>0 else np.nan
    else:
        A_pkts_est_total = np.nan
        A_pkts_est_avg   = np.nan
        A_size_est       = np.nan

    # Arrival LB (참고)
    A_bytes_lb_total = float(sub["arrival_bytes_lb"].sum())
    A_bytes_lb_avg   = A_bytes_lb_total / N
    A_size_lb        = (A_bytes_lb_total / A_pkts_est_total) if (isinstance(A_pkts_est_total, float) and A_pkts_est_total>0) else np.nan

    # backlog_bytes 통계 (윈도우)
    B = sub["backlog_bytes"]
    B_avg  = float(B.mean())
    B_med  = float(B.median())
    B_min  = float(B.min())
    B_max  = float(B.max())
    dB     = sub["backlog_delta_bytes"]
    dB_pos = float(dB.clip(lower=0).sum())
    dB_neg = float(dB.clip(upper=0).sum())
    dB_net = float(dB.sum())

    return {
        "slots": int(N),

        # Service (처리)
        "service_pkts_avg_per_slot": S_pkts_avg,
        "service_bytes_avg_per_slot": S_bytes_avg,
        "service_size_bytes_per_pkt": S_size,

        # Arrival EST (상위)
        "arrival_pkts_est_avg_per_slot": A_pkts_est_avg,
        "arrival_bytes_est_avg_per_slot": A_bytes_est_avg,
        "arrival_size_est_bytes_per_pkt": A_size_est,

        # Arrival LB (참고)
        "arrival_bytes_lb_avg_per_slot": A_bytes_lb_avg,
        "arrival_size_lb_bytes_per_pkt": A_size_lb,

        # backlog_bytes (윈도우)
        "backlog_bytes_avg_per_slot": B_avg,
        "backlog_bytes_median_per_slot": B_med,
        "backlog_bytes_min_per_slot": B_min,
        "backlog_bytes_max_per_slot": B_max,
        "backlog_bytes_pos_delta_total": dB_pos,
        "backlog_bytes_neg_delta_total": dB_neg,
        "backlog_bytes_net_delta_total": dB_net,
    }

# ---------- batch ----------
def expand_files(prefix: Optional[str], start: int, end: int, ext: str) -> List[Path]:
    out=[]
    if not prefix: return out
    if "{i}" not in prefix and "{idx}" not in prefix:
        raise ValueError("--prefix에는 {i} 또는 {idx} 필요")
    for i in range(start, end+1):
        p = Path(prefix.format(i=i, idx=i))
        if p.suffix=="" and ext: p = p.with_suffix(ext)
        out.append(p)
    return out

def analyze_many(files: List[Path], glob_pat: Optional[str], neighbors: int) -> pd.DataFrame:
    paths=[]
    if glob_pat: paths.extend(sorted(Path().glob(glob_pat)))
    paths.extend(files)

    uniq, seen=[], set()
    for p in paths:
        q=p
        if not q.exists() and q.suffix=="": q=q.with_suffix(".csv")
        if not q.exists():
            gz=Path(str(q)+".gz")
            if gz.exists(): q=gz
        if q.exists():
            r=q.resolve()
            if r not in seen:
                uniq.append(r); seen.add(r)
        else:
            print(f"[skip] not found: {p}", file=sys.stderr)

    rows=[]
    for p in uniq:
        try:
            df = load_df(p)
            rec = per_file_window_averages(df, neighbors=neighbors)
            rec.update({"file": p.name})
        except Exception as e:
            rec = {"file": p.name, "error": str(e), "slots": 0}
        rows.append(rec)

    return pd.DataFrame(rows)

# ---------- summary ----------
def summarize_over_files(df: pd.DataFrame, cols: List[str], title: str):
    sel = df[df["slots"]>0]
    print(f"\n=== {title} ===")
    if sel.empty:
        print("(no drop windows)"); return
    stats={}
    for c in cols:
        s = pd.to_numeric(sel[c], errors="coerce")
        stats[c] = {
            "mean": float(s.mean()),
            "median": float(s.median()),
            "std": float(s.std()),
            "min": float(s.min()),
            "max": float(s.max()),
        }
    out = pd.DataFrame([{"metric": k, **v} for k,v in stats.items()])
    with pd.option_context("display.width", 160, "display.precision", 2):
        print(out.to_string(index=False))

# ---------- CLI ----------
def main():
    ap = argparse.ArgumentParser(description="Burst(drop) window averages — service vs arrival (pkts/bytes/size)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--glob", help='예: "/mnt/data/mac_aqm_*_13_*_H4_P0(NP).csv"')
    g.add_argument("--prefix", help='예: "/mnt/data/..._13_{i}_H4_P0(NP)" (여기 {i}|{idx})')
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--end",   type=int, default=150)
    ap.add_argument("--ext", default=".csv")
    ap.add_argument("--neighbors", type=int, default=1, help="드롭 슬롯 ±이웃 합집합 폭 (기본 1)")

    # 저장 옵션
    ap.add_argument("--out-csv", default="", help="파일별 결과를 CSV로 저장할 경로 (예: results.csv)")
    ap.add_argument("--out-cols", default="", help="저장할 컬럼만 콤마로 지정 (예: service_bytes_avg_per_slot,backlog_bytes_avg_per_slot)")

    args = ap.parse_args()

    files = expand_files(args.prefix, args.start, args.end, args.ext) if args.prefix else []
    df = analyze_many(files, args.glob, neighbors=args.neighbors)

    # 파일별 표
    cols = ["file","slots",
            "service_pkts_avg_per_slot","service_bytes_avg_per_slot","service_size_bytes_per_pkt",
            "arrival_pkts_est_avg_per_slot","arrival_bytes_est_avg_per_slot","arrival_size_est_bytes_per_pkt",
            "arrival_bytes_lb_avg_per_slot","arrival_size_lb_bytes_per_pkt",
            "backlog_bytes_avg_per_slot","backlog_bytes_median_per_slot",
            "backlog_bytes_min_per_slot","backlog_bytes_max_per_slot",
            "backlog_bytes_pos_delta_total","backlog_bytes_neg_delta_total","backlog_bytes_net_delta_total",
            "error"]
    avail = [c for c in cols if c in df.columns]
    print("\n=== 파일별 — 드롭 윈도우 평균 (pkts/0.1s, bytes/0.1s, bytes/pkt, backlog) ===")
    with pd.option_context("display.max_colwidth", 120, "display.width", 180):
        print(df[avail].to_string(index=False))

    # 전체 요약
    sum_cols = [
        "service_pkts_avg_per_slot","service_bytes_avg_per_slot","service_size_bytes_per_pkt",
        "arrival_pkts_est_avg_per_slot","arrival_bytes_est_avg_per_slot","arrival_size_est_bytes_per_pkt",
        "arrival_bytes_lb_avg_per_slot","arrival_size_lb_bytes_per_pkt",
        "backlog_bytes_avg_per_slot","backlog_bytes_median_per_slot",
        "backlog_bytes_min_per_slot","backlog_bytes_max_per_slot",
        "backlog_bytes_pos_delta_total","backlog_bytes_neg_delta_total","backlog_bytes_net_delta_total",
    ]
    summarize_over_files(df, sum_cols, title="전체 요약 — 파일별 평균의 통계")

    # CSV 저장
    if args.out_csv:
        to_save = df.copy()
        if args.out_cols.strip():
            keep = [c.strip() for c in args.out_cols.split(",") if c.strip()]
            if "file" not in keep: keep = ["file"] + keep
            missing = [c for c in keep if c not in to_save.columns]
            if missing:
                print(f"[WARN] missing columns in out-cols: {missing}", file=sys.stderr)
            to_save = to_save[[c for c in keep if c in to_save.columns]]
        else:
            # 기본 저장 컬럼(핵심 + backlog 평균)
            keep = ["file","slots","service_bytes_avg_per_slot",
                    "arrival_bytes_est_avg_per_slot",
                    "backlog_bytes_avg_per_slot","backlog_bytes_median_per_slot",
                    "backlog_bytes_pos_delta_total","backlog_bytes_net_delta_total"]
            to_save = to_save[[c for c in keep if c in to_save.columns]]

        out_path = Path(args.out_csv).expanduser().resolve()
        to_save.to_csv(out_path, index=False)
        print(f"\n[INFO] saved file-level results to: {out_path}")

if __name__ == "__main__":
    main()

