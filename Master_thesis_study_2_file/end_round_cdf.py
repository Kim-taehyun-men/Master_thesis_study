# -*- coding: utf-8 -*-
"""
End-round probability measurement for Fast-DDS discovery HB CSVs.

Each CSV has two columns (no header):
- col0: round number (int)
- col1: always 1 (ignored)

Rule:
- For each file, the "end round" is the maximum value of the first column.
- If a file ended at round r, then it is also considered "finished by t" for all t >= r.
  → We compute both P(end == t) and the cumulative P(end ≤ t).

Usage example:
    python3 end_round_cdf.py \
        --prefix "/mnt/data/fastdds_discovery_hb_" \
        --start 1 --end 100 \
        --ext ".csv" \
        --max-threshold 10 \
        --out-dir "/mnt/data"

Outputs:
- end_cdf_summary.csv : table with threshold, successes(≤t), total_files, p_leq_t, p_exact_t
- end_round_per_file.csv : per-file end round results
- end_cdf_plot.png : line chart of P(end ≤ t)
- end_pmf_plot.png : bar chart of P(end == t)
"""

import os
import sys
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def read_end_round_from_file(path: str):
    """Return the maximum of the first column as the end round, or None if not parseable."""
    try:
        # Accept comma or whitespace separators
        df = pd.read_csv(
            path,
            sep=r"[,\s]+",
            header=None,
            names=["round", "flag"],
            engine="python",
            comment="#"
        )
        if "round" not in df.columns or df["round"].dropna().empty:
            return None
        rounds = pd.to_numeric(df["round"], errors="coerce").dropna().astype(int)
        if rounds.empty:
            return None
        return int(rounds.max())
    except Exception as e:
        print(f"[WARN] Failed to read: {path} ({e})")
        return None

def compute_probabilities(end_rounds, max_threshold: int):
    """Compute exact and cumulative probabilities up to max_threshold."""
    total = len(end_rounds)
    exact_counts = {t: 0 for t in range(1, max_threshold + 1)}
    for r in end_rounds:
        if 1 <= r <= max_threshold:
            exact_counts[r] += 1
    cumulative_counts = {}
    running = 0
    for t in range(1, max_threshold + 1):
        running += exact_counts.get(t, 0)
        cumulative_counts[t] = running
    # Build summary rows
    summary_rows = []
    for t in range(1, max_threshold + 1):
        successes = cumulative_counts[t]
        p_leq_t = successes / total if total > 0 else float("nan")
        p_exact_t = exact_counts[t] / total if total > 0 else float("nan")
        summary_rows.append({
            "threshold": t,
            "successes(≤t)": successes,
            "total_files": total,
            "p_leq_t": p_leq_t,
            "p_exact_t": p_exact_t,
        })
    return pd.DataFrame(summary_rows)

def save_plots(df_summary, out_dir: str):
    # CDF plot P(end ≤ t)
    plt.figure(figsize=(7, 4.5))
    plt.plot(df_summary["threshold"], df_summary["p_leq_t"], marker="o")
    plt.title("Cumulative Probability P(end ≤ t)")
    plt.xlabel("t (threshold)")
    plt.ylabel("Probability")
    plt.xticks(list(df_summary["threshold"]))
    plt.ylim(0, 1)
    plt.grid(True, linestyle="--", alpha=0.6)
    cdf_path = os.path.join(out_dir, "end_cdf_plot.png")
    plt.tight_layout()
    plt.savefig(cdf_path, dpi=150)
    plt.close()

    # PMF plot P(end == t)
    plt.figure(figsize=(7, 4.5))
    plt.bar(df_summary["threshold"], df_summary["p_exact_t"])
    plt.title("Exact Probability P(end == t)")
    plt.xlabel("t (threshold)")
    plt.ylabel("Probability")
    plt.xticks(list(df_summary["threshold"]))
    plt.ylim(0, 1)
    plt.grid(True, axis="y", linestyle="--", alpha=0.6)
    pmf_path = os.path.join(out_dir, "end_pmf_plot.png")
    plt.tight_layout()
    plt.savefig(pmf_path, dpi=150)
    plt.close()
    return cdf_path, pmf_path

def main():
    ap = argparse.ArgumentParser(description="Measure end-round probabilities from Fast-DDS discovery HB CSVs.")
    ap.add_argument("--prefix", type=str, default="/mnt/data/fastdds_discovery_hb_",
                    help="File prefix, e.g., '/path/fastdds_discovery_hb_'")
    ap.add_argument("--start", type=int, default=1, help="Start index (inclusive)")
    ap.add_argument("--end", type=int, default=100, help="End index (inclusive)")
    ap.add_argument("--ext", type=str, default=".csv", help="File extension (with dot)")
    ap.add_argument("--max-threshold", type=int, default=10, help="Max threshold (t in 1..T)")
    ap.add_argument("--out-dir", type=str, default="/mnt/data", help="Output directory")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Collect per-file end rounds
    records = []
    for i in range(args.start, args.end + 1):
        path = f"{args.prefix}{i}{args.ext}"
        if not os.path.exists(path):
            continue
        end_round = read_end_round_from_file(path)
        if end_round is not None:
            records.append({"file_index": i, "end_round": end_round})

    if not records:
        print("No files processed. Check --prefix/--ext and index range.")
        sys.exit(1)

    df_files = pd.DataFrame(records).sort_values("file_index").reset_index(drop=True)
    total_files = len(df_files)
    end_rounds = df_files["end_round"].tolist()

    # Compute probabilities
    df_summary = compute_probabilities(end_rounds, args.max_threshold)

    # Save CSVs
    summary_csv_path = os.path.join(args.out_dir, "end_cdf_summary.csv")
    perfile_csv_path = os.path.join(args.out_dir, "end_round_per_file.csv")
    df_summary.to_csv(summary_csv_path, index=False, encoding="utf-8")
    df_files.to_csv(perfile_csv_path, index=False, encoding="utf-8")

    # Save plots
    cdf_path, pmf_path = save_plots(df_summary, args.out_dir)

    # Console report
    print(f"[Done] Files processed: {total_files}")
    print(f"- Summary CSV: {summary_csv_path}")
    print(f"- Per-file CSV: {perfile_csv_path}")
    print(f"- CDF plot: {cdf_path}")
    print(f("- PMF plot: {pmf_path}"))

if __name__ == "__main__":
    main()
