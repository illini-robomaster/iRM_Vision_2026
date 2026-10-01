#!/usr/bin/env python3
"""对比两份 `detect_freq_visual_test -dump=CSV` 的检测结果（A/B 一致性基线）。

对应 AGENTS.md §4.1.4「合入前必须给出与参考实现的一致性对比」：以前靠人工看数字，
现在把口径固定下来，换成可复跑的脚本（scripts/replay_ab.sh 调它）。

用法：
  python3 scripts/csv_ab.py A.csv B.csv [--name-a=ort] [--name-b=dnn]
                         [--max-conf-diff=1e-2] [--max-px-diff=3.0]
                         [--max-label-mismatch=0] [--examples=3]

CSV 表头（detect_freq_visual_test 的 -dump 写出，每帧每个装甲板一行）：
  frame,armor_idx,confidence,color,name,type,cx,cy,x0,y0,x1,y1,x2,y2,x3,y3

比较口径：
  1. 同帧内按「中心距离最小」贪心配对——**不能按 armor_idx 配**：那是各后端 NMS 之后的顺序，
     换后端可能顺序不同，按序号直接配会误报；
  2. 标签（color / name / type）不一致条数 <= --max-label-mismatch（默认 0）；
  3. max|Δconfidence| <= --max-conf-diff（默认 1e-2，给 TensorRT FP16 留余量）；
     max|Δ角点像素|（4 点共 8 个坐标） <= --max-px-diff（默认 3.0）。
另外「有检出的帧数」「单侧检出帧（一边有、一边没有）」「装甲板数不一致的帧」都会打印，
它们不为 0 即判 FAIL：这正是 A/B 想发现的差异（例如调 min_confidence 之后）。

退出码：0 = 通过；1 = 超差 / 不可比（打印样例）；2 = 文件或格式问题。
"""

import argparse
import csv
import math
import sys
from collections import defaultdict

POINT_KEYS = ("x0", "y0", "x1", "y1", "x2", "y2", "x3", "y3")
LABEL_KEYS = ("color", "name", "type")


def load(path):
    """读成 {frame: [row, ...]}，保持文件内顺序。"""
    frames = defaultdict(list)
    keys = ("frame", "confidence", "cx", "cy") + LABEL_KEYS
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise ValueError("空文件（没有表头）")
            missing = [k for k in keys if k not in reader.fieldnames]
            if missing:
                raise ValueError("缺少列 " + ", ".join(missing))
            for row in reader:
                frames[int(float(row["frame"]))].append(row)
    except OSError as e:
        print(f"[csv_ab] 无法读取 {path}: {e}")
        sys.exit(2)
    except (KeyError, ValueError) as e:
        print(f"[csv_ab] {path} 不是 -dump 生成的 CSV: {e}")
        sys.exit(2)
    return frames


def center_dist(a, b):
    return math.hypot(float(a["cx"]) - float(b["cx"]), float(a["cy"]) - float(b["cy"]))


def label_of(row):
    return tuple(row[k] for k in LABEL_KEYS)


def px_diff(a, b):
    return max(abs(float(a[k]) - float(b[k])) for k in POINT_KEYS)


def pair_rows(rows_a, rows_b):
    """贪心最小中心距离配对，返回 (pairs, 未配对的 A 行, 未配对的 B 行)。"""
    cands = sorted(
        (center_dist(a, b), i, j) for i, a in enumerate(rows_a) for j, b in enumerate(rows_b)
    )
    free_a = set(range(len(rows_a)))
    free_b = set(range(len(rows_b)))
    pairs = []
    for _, i, j in cands:
        if i in free_a and j in free_b:
            free_a.discard(i)
            free_b.discard(j)
            pairs.append((rows_a[i], rows_b[j]))
    return pairs, [rows_a[i] for i in sorted(free_a)], [rows_b[j] for j in sorted(free_b)]


def percentile(values, p):
    if not values:
        return 0.0
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p * (len(s) - 1))))]



def main():
    ap = argparse.ArgumentParser(description="检测结果 A/B 一致性对比（两份 -dump CSV 之间）")
    ap.add_argument("csv_a")
    ap.add_argument("csv_b")
    ap.add_argument("--name-a", default="A")
    ap.add_argument("--name-b", default="B")
    ap.add_argument("--max-conf-diff", type=float, default=1e-2)
    ap.add_argument("--max-px-diff", type=float, default=3.0)
    ap.add_argument("--max-label-mismatch", type=int, default=0)
    ap.add_argument("--examples", type=int, default=3, help="每个失败类别打印多少条样例")
    args = ap.parse_args()

    frames_a = load(args.csv_a)
    frames_b = load(args.csv_b)
    frames = sorted(set(frames_a) | set(frames_b))

    n_a = sum(1 for f in frames if frames_a[f])
    n_b = sum(1 for f in frames if frames_b[f])
    both = [f for f in frames if frames_a[f] and frames_b[f]]
    only_a = [f for f in frames if frames_a[f] and not frames_b[f]]
    only_b = [f for f in frames if frames_b[f] and not frames_a[f]]

    count_mismatch = []  # (frame, na, nb)
    label_mismatch = []  # (frame, label_a, label_b)
    unpaired = []  # (frame, 哪一侧, (cx, cy))
    conf_diffs = []
    px_diffs = []
    worst_conf = None  # (diff, frame, conf_a, conf_b)
    worst_px = None  # (diff, frame, 行 A, 行 B)

    for f in both:
        ra, rb = frames_a[f], frames_b[f]
        if len(ra) != len(rb):
            count_mismatch.append((f, len(ra), len(rb)))
        pairs, left_a, left_b = pair_rows(ra, rb)
        for row in left_a:
            unpaired.append((f, args.name_a, (row["cx"], row["cy"])))
        for row in left_b:
            unpaired.append((f, args.name_b, (row["cx"], row["cy"])))
        for a, b in pairs:
            if label_of(a) != label_of(b):
                label_mismatch.append((f, label_of(a), label_of(b)))
            dc = abs(float(a["confidence"]) - float(b["confidence"]))
            dp = px_diff(a, b)
            conf_diffs.append(dc)
            px_diffs.append(dp)
            if worst_conf is None or dc > worst_conf[0]:
                worst_conf = (dc, f, a["confidence"], b["confidence"])
            if worst_px is None or dp > worst_px[0]:
                worst_px = (dp, f, a, b)

    max_conf = max(conf_diffs) if conf_diffs else 0.0
    max_px = max(px_diffs) if px_diffs else 0.0
    p95_conf = percentile(conf_diffs, 0.95)
    p95_px = percentile(px_diffs, 0.95)

    print(f"==== CSV A/B: {args.name_a} vs {args.name_b} ====")
    print(f"  {args.name_a}: {args.csv_a}")
    print(f"  {args.name_b}: {args.csv_b}")
    print(f"  有检出的帧数: {args.name_a} {n_a} / {args.name_b} {n_b} / 共同 {len(both)}")
    print(
        f"  单侧检出帧: {args.name_a} {len(only_a)} / {args.name_b} {len(only_b)}"
        f"；装甲板数不一致: {len(count_mismatch)} 帧；未配对条目: {len(unpaired)}"
    )
    print(f"  标签(color/name/type)不一致: {len(label_mismatch)} 条")
    print(f"  |Δconfidence|: max {max_conf:.2e} (p95 {p95_conf:.2e}, n={len(conf_diffs)})")
    print(f"  |Δpx|(角点8坐标最大): max {max_px:.3f} (p95 {p95_px:.3f})")
    print(
        f"  阈值: conf {args.max_conf_diff:.1e} / px {args.max_px_diff:.2f} / "
        f"标签不一致 {args.max_label_mismatch}"
    )

    fails = []
    if not frames_a or not frames_b:
        fails.append(f"{args.name_a} 或 {args.name_b} 完全没有检出，无法比较")
    else:
        if not both:
            fails.append("两边没有共同检出帧，无法比较")
        if only_a or only_b:
            fails.append(
                f"存在单侧检出帧（{args.name_a} {len(only_a)} / {args.name_b} {len(only_b)}）"
            )
        if count_mismatch:
            fails.append(f"存在装甲板数不一致的帧（{len(count_mismatch)}）")
        if unpaired:
            fails.append(f"存在未配对条目（{len(unpaired)}）")
        if len(label_mismatch) > args.max_label_mismatch:
            fails.append(f"标签不一致 {len(label_mismatch)} 条 > {args.max_label_mismatch}")
        if max_conf > args.max_conf_diff:
            fails.append(f"max|Δconf| {max_conf:.2e} > {args.max_conf_diff:.2e}")
        if max_px > args.max_px_diff:
            fails.append(f"max|Δpx| {max_px:.3f} > {args.max_px_diff:.2f}")

    n = args.examples
    if fails and n > 0:
        print("  --- 差异样例 ---")
        if only_a or only_b:
            print(f"  单侧检出帧: {args.name_a} {only_a[:n]} / {args.name_b} {only_b[:n]}")
        for f, na, nb in count_mismatch[:n]:
            print(f"  frame {f}: 装甲板数 {args.name_a}={na} vs {args.name_b}={nb}")
        for f, la, lb in label_mismatch[:n]:
            print(f"  frame {f}: 标签 {la} vs {lb}")
        for f, side, c in unpaired[:n]:
            print(f"  frame {f}: {side} 未配对 (cx,cy)=({c[0]},{c[1]})")
        if worst_conf is not None:
            print(
                f"  最大 |Δconf|: frame {worst_conf[1]} {args.name_a}={float(worst_conf[2]):.5f} "
                f"vs {args.name_b}={float(worst_conf[3]):.5f}"
            )
        if worst_px is not None:
            a, b = worst_px[2], worst_px[3]
            print(
                f"  最大 |Δpx|: frame {worst_px[1]} {worst_px[0]:.3f}px "
                f"{args.name_a}(cx,cy)=({a['cx']},{a['cy']}) vs "
                f"{args.name_b}(cx,cy)=({b['cx']},{b['cy']})"
            )

    for reason in fails:
        print(f"  FAIL: {reason}")
    print("CSV_AB PASS" if not fails else "CSV_AB FAIL")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
