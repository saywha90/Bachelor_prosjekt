"""
run_benchmark.py
================

Automatisert ytelsestest for SimpleBallDetector.

Kjører en tidsbestemt session mot live OAK-kamera, samler per-frame metrics
(FPS, antall detekterte baller, confidence, deteksjonsmetode) og lagrer
rå CSV-data + ferdigrendrerte grafer til samme mappe.

Bruk:
    python tests/vision_benchmark/run_benchmark.py

Output (i tests/vision_benchmark/results/):
    raw_data.csv          — per-frame rådata
    fps_over_time.png     — FPS over tid
    detections_per_frame.png — antall detekterte baller per frame
    confidence_dist.png   — confidence-fordeling per farge
    method_breakdown.png  — deteksjonsmetode-fordeling (kakediagram)
    summary.json          — maskinlesbar oppsummering
"""

import sys
import time
import json
import csv
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Headless – ingen GUI-vindu for matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

# ── Paths ──────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from vision.oak_camera import OAKCamera
from vision.enhanced_detector import SimpleBallDetector, BallColor
import config

RESULTS_DIR = Path(__file__).parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)

# ── Benchmark parameters ───────────────────────────────────────────────────
BENCHMARK_DURATION_S  = 30    # Sekunder med opptak
EXPECTED_RED_BALLS    = 2     # Antall røde baller i scenen (juster ved behov)
EXPECTED_BLUE_BALLS   = 2     # Antall blå baller i scenen (juster ved behov)

# ── Fargepalett (matplotlib) ───────────────────────────────────────────────
RED_C  = "#E53935"
BLUE_C = "#1E88E5"
GRAY_C = "#757575"
BG_C   = "#F5F5F5"

plt.rcParams.update({
    "font.family":    "DejaVu Sans",
    "font.size":      11,
    "axes.spines.top":   False,
    "axes.spines.right": False,
    "axes.facecolor": BG_C,
    "figure.facecolor": "white",
})


# ═══════════════════════════════════════════════════════════════════════════
# Data collection
# ═══════════════════════════════════════════════════════════════════════════

def run_capture(duration_s: int) -> list[dict]:
    """
    Kjører kamera + detektor i 'duration_s' sekunder og returnerer
    en liste med per-frame metadata.
    """
    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)
    if not cam.open():
        print("FEIL: Klarte ikke åpne OAK-kameraet.")
        sys.exit(1)

    focal_px = cam.get_focal_length_px(hfov_deg=config.CAMERA_HFOV_DEG)
    print(f"OK Brennvidde: {focal_px:.1f} px  (HFOV={config.CAMERA_HFOV_DEG}°)")

    detector = SimpleBallDetector(
        min_radius=10,
        max_radius=150,
        confidence_threshold=0.35,
        enable_adaptive_lighting=True,
        max_balls_per_color=4,
        focal_length_px=focal_px,
    )

    print(f"\n▶  Samler data i {duration_s} sekunder …  (trykk ikke noe)")

    records = []
    frame_idx = 0
    t_start = time.perf_counter()
    t_fps   = t_start
    fps_buf = []

    while True:
        elapsed = time.perf_counter() - t_start
        if elapsed >= duration_s:
            break

        ret, frame = cam.read()
        if not ret:
            continue

        t0 = time.perf_counter()
        balls, stats = detector.detect_balls(frame)
        proc_ms = (time.perf_counter() - t0) * 1000

        fps_buf.append(time.perf_counter())
        # Beregn FPS over siste sekund
        fps_buf = [t for t in fps_buf if time.perf_counter() - t <= 1.0]
        fps = len(fps_buf)

        for ball in balls:
            records.append({
                "frame":     frame_idx,
                "elapsed_s": round(elapsed, 4),
                "fps":       fps,
                "proc_ms":   round(proc_ms, 2),
                "color":     ball.color.value,
                "cx":        ball.center[0],
                "cy":        ball.center[1],
                "radius_px": round(ball.radius, 1),
                "confidence":round(ball.confidence, 4),
                "method":    ball.detection_method,
                "dist_cm":   ball.distance_cm if ball.distance_cm else "",
            })

        # Hvis ingen baller detektert — logg tom rad for FPS-sporing
        if not balls:
            records.append({
                "frame":     frame_idx,
                "elapsed_s": round(elapsed, 4),
                "fps":       fps,
                "proc_ms":   round(proc_ms, 2),
                "color":     "none",
                "cx":        "",
                "cy":        "",
                "radius_px": "",
                "confidence":"",
                "method":    "none",
                "dist_cm":   "",
            })

        frame_idx += 1
        # Vis progress hvert 5. sekund
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            pct = int(elapsed / duration_s * 100)
            print(f"   {pct:3d}%  ({int(elapsed)}/{duration_s}s)  FPS={fps}", end="\r")

    cam.release()
    print(f"\n✓  Ferdig. {frame_idx} frames fanget.")
    return records


# ═══════════════════════════════════════════════════════════════════════════
# CSV export
# ═══════════════════════════════════════════════════════════════════════════

def save_csv(records: list[dict]) -> Path:
    path = RESULTS_DIR / "raw_data.csv"
    if not records:
        return path
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    print(f"✓  CSV lagret → {path.relative_to(ROOT)}")
    return path


# ═══════════════════════════════════════════════════════════════════════════
# Derived metrics
# ═══════════════════════════════════════════════════════════════════════════

def compute_metrics(records: list[dict]) -> dict:
    # Unike frames
    frames_all = {}
    for r in records:
        fi = r["frame"]
        if fi not in frames_all:
            frames_all[fi] = {"fps": r["fps"], "proc_ms": r["proc_ms"],
                               "elapsed_s": r["elapsed_s"],
                               "red": 0, "blue": 0, "conf_red": [], "conf_blue": []}
        if r["color"] == "red":
            frames_all[fi]["red"] += 1
            if r["confidence"] != "":
                frames_all[fi]["conf_red"].append(float(r["confidence"]))
        elif r["color"] == "blue":
            frames_all[fi]["blue"] += 1
            if r["confidence"] != "":
                frames_all[fi]["conf_blue"].append(float(r["confidence"]))

    frame_list = sorted(frames_all.values(), key=lambda x: x["elapsed_s"])

    fps_series     = [f["fps"]       for f in frame_list]
    proc_series    = [f["proc_ms"]   for f in frame_list]
    red_series     = [f["red"]       for f in frame_list]
    blue_series    = [f["blue"]      for f in frame_list]
    time_series    = [f["elapsed_s"] for f in frame_list]

    conf_red  = [c for f in frame_list for c in f["conf_red"]]
    conf_blue = [c for f in frame_list for c in f["conf_blue"]]

    method_counts = {}
    for r in records:
        m = r["method"]
        method_counts[m] = method_counts.get(m, 0) + 1

    n = len(frame_list)
    return {
        "n_frames":        n,
        "duration_s":      round(max(time_series) if time_series else 0, 2),
        "fps_mean":        round(np.mean(fps_series), 1),
        "fps_min":         round(np.min(fps_series), 1),
        "fps_max":         round(np.max(fps_series), 1),
        "fps_std":         round(np.std(fps_series), 2),
        "proc_mean_ms":    round(np.mean(proc_series), 2),
        "proc_p95_ms":     round(np.percentile(proc_series, 95), 2),
        "red_mean":        round(np.mean(red_series), 3),
        "blue_mean":       round(np.mean(blue_series), 3),
        "red_det_rate":    round(np.mean([1 if r > 0 else 0 for r in red_series]) * 100, 1),
        "blue_det_rate":   round(np.mean([1 if b > 0 else 0 for b in blue_series]) * 100, 1),
        "conf_red_mean":   round(np.mean(conf_red)  if conf_red  else 0, 3),
        "conf_blue_mean":  round(np.mean(conf_blue) if conf_blue else 0, 3),
        "conf_red_list":   conf_red,
        "conf_blue_list":  conf_blue,
        "method_counts":   method_counts,
        "time_series":     time_series,
        "fps_series":      fps_series,
        "proc_series":     proc_series,
        "red_series":      red_series,
        "blue_series":     blue_series,
    }


# ═══════════════════════════════════════════════════════════════════════════
# Plots
# ═══════════════════════════════════════════════════════════════════════════

def _savefig(fig: plt.Figure, name: str) -> Path:
    path = RESULTS_DIR / name
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"✓  Graf lagret → {path.relative_to(ROOT)}")
    return path


def plot_fps(m: dict):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(m["time_series"], m["fps_series"], color=GRAY_C, linewidth=1.2, alpha=0.6)
    # 1-sekund glidende gjennomsnitt
    if len(m["fps_series"]) > 10:
        kernel = np.ones(10) / 10
        smooth = np.convolve(m["fps_series"], kernel, mode="valid")
        t_sm   = m["time_series"][:len(smooth)]
        ax.plot(t_sm, smooth, color="#37474F", linewidth=2.0, label="Glidende snitt (10 frames)")
    ax.axhline(m["fps_mean"], color="#F57C00", linewidth=1.5,
               linestyle="--", label=f"Gjennomsnitt: {m['fps_mean']} FPS")
    ax.set_xlabel("Tid (s)")
    ax.set_ylabel("FPS")
    ax.set_title("Bildebehandlingshastighet over tid", fontweight="bold", pad=12)
    ax.legend(framealpha=0.85, fontsize=10)
    ax.set_ylim(bottom=0)
    ax.set_xlim(left=0)
    fig.tight_layout()
    return _savefig(fig, "fps_over_time.png")


def plot_detections(m: dict):
    fig, ax = plt.subplots(figsize=(10, 4))
    t = m["time_series"]
    ax.fill_between(t, m["red_series"],  alpha=0.35, color=RED_C,  step="mid")
    ax.fill_between(t, m["blue_series"], alpha=0.35, color=BLUE_C, step="mid")
    ax.step(t, m["red_series"],  color=RED_C,  linewidth=1.5, label="Røde baller")
    ax.step(t, m["blue_series"], color=BLUE_C, linewidth=1.5, label="Blå baller")
    ax.axhline(EXPECTED_RED_BALLS,  color=RED_C,  linewidth=1.0,
               linestyle=":", alpha=0.6, label=f"Forventet rød: {EXPECTED_RED_BALLS}")
    ax.axhline(EXPECTED_BLUE_BALLS, color=BLUE_C, linewidth=1.0,
               linestyle=":", alpha=0.6, label=f"Forventet blå: {EXPECTED_BLUE_BALLS}")
    ax.set_xlabel("Tid (s)")
    ax.set_ylabel("Antall detekterte baller")
    ax.set_title("Detekterte baller per frame over tid", fontweight="bold", pad=12)
    ax.legend(framealpha=0.85, fontsize=10)
    ax.set_ylim(bottom=0)
    ax.set_xlim(left=0)
    ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
    fig.tight_layout()
    return _savefig(fig, "detections_per_frame.png")


def plot_confidence(m: dict):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, conf, color, label in [
        (axes[0], m["conf_red_list"],  RED_C,  "Røde baller"),
        (axes[1], m["conf_blue_list"], BLUE_C, "Blå baller"),
    ]:
        if conf:
            ax.hist(conf, bins=20, range=(0, 1), color=color,
                    edgecolor="white", linewidth=0.6, alpha=0.85)
            ax.axvline(np.mean(conf), color="#37474F", linewidth=1.8,
                       linestyle="--",
                       label=f"Gjennomsnitt: {np.mean(conf):.2f}")
            ax.legend(framealpha=0.85, fontsize=10)
        ax.set_xlabel("Confidence-score")
        ax.set_title(label, fontweight="bold")
        ax.set_xlim(0, 1)
    axes[0].set_ylabel("Antall deteksjoner")
    fig.suptitle("Fordeling av confidence-score per fargeklasse",
                 fontweight="bold", fontsize=13, y=1.01)
    fig.tight_layout()
    return _savefig(fig, "confidence_dist.png")


def plot_method_breakdown(m: dict):
    counts = {k: v for k, v in m["method_counts"].items() if k != "none"}
    if not counts:
        return
    labels = list(counts.keys())
    values = list(counts.values())
    colors = ["#42A5F5", "#EF5350", "#66BB6A", "#FFA726"][:len(labels)]

    fig, ax = plt.subplots(figsize=(6, 6))
    wedges, texts, autotexts = ax.pie(
        values, labels=None, colors=colors,
        autopct="%1.1f%%", startangle=90,
        wedgeprops={"edgecolor": "white", "linewidth": 2},
        pctdistance=0.78,
    )
    for at in autotexts:
        at.set_fontsize(11)
        at.set_fontweight("bold")
        at.set_color("white")
    ax.legend(wedges, [l.capitalize() for l in labels],
              loc="lower center", bbox_to_anchor=(0.5, -0.08),
              ncol=len(labels), fontsize=10, framealpha=0.85)
    ax.set_title("Deteksjonsmetode-fordeling", fontweight="bold", pad=14)
    fig.tight_layout()
    return _savefig(fig, "method_breakdown.png")


def plot_processing_latency(m: dict):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(m["time_series"], m["proc_series"], color="#5C6BC0",
            linewidth=1.0, alpha=0.5)
    if len(m["proc_series"]) > 10:
        kernel = np.ones(10) / 10
        smooth = np.convolve(m["proc_series"], kernel, mode="valid")
        t_sm   = m["time_series"][:len(smooth)]
        ax.plot(t_sm, smooth, color="#283593", linewidth=2.0,
                label="Glidende snitt (10 frames)")
    ax.axhline(m["proc_mean_ms"], color="#F57C00", linewidth=1.5,
               linestyle="--", label=f"Snitt: {m['proc_mean_ms']} ms")
    ax.axhline(m["proc_p95_ms"], color="#C62828", linewidth=1.5,
               linestyle="--", label=f"P95: {m['proc_p95_ms']} ms")
    ax.set_xlabel("Tid (s)")
    ax.set_ylabel("Behandlingstid (ms)")
    ax.set_title("Latens per frame over tid", fontweight="bold", pad=12)
    ax.legend(framealpha=0.85, fontsize=10)
    ax.set_ylim(bottom=0)
    ax.set_xlim(left=0)
    fig.tight_layout()
    return _savefig(fig, "latency_over_time.png")


# ═══════════════════════════════════════════════════════════════════════════
# Summary JSON
# ═══════════════════════════════════════════════════════════════════════════

def _to_py(v):
    """Konverter numpy-skalarer til native Python-typer for JSON-serialisering."""
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.floating,)):
        return float(v)
    if isinstance(v, dict):
        return {k: _to_py(vv) for k, vv in v.items()}
    return v


def save_summary(m: dict):
    summary = {
        "test_timestamp":        datetime.now().isoformat(),
        "camera":                "Luxonis OAK Series 2 (IMX378)",
        "resolution":            f"{config.CAMERA_RESOLUTION[0]}x{config.CAMERA_RESOLUTION[1]}",
        "usb_connection":        "USB 2.0 via Dell-adapter",
        "duration_s":            m["duration_s"],
        "n_frames":              m["n_frames"],
        "fps_mean":              m["fps_mean"],
        "fps_min":               m["fps_min"],
        "fps_max":               m["fps_max"],
        "fps_std":               m["fps_std"],
        "latency_mean_ms":       m["proc_mean_ms"],
        "latency_p95_ms":        m["proc_p95_ms"],
        "expected_red_balls":    EXPECTED_RED_BALLS,
        "expected_blue_balls":   EXPECTED_BLUE_BALLS,
        "avg_detected_red":      m["red_mean"],
        "avg_detected_blue":     m["blue_mean"],
        "detection_rate_red_pct":  m["red_det_rate"],
        "detection_rate_blue_pct": m["blue_det_rate"],
        "confidence_mean_red":   m["conf_red_mean"],
        "confidence_mean_blue":  m["conf_blue_mean"],
        "method_counts":         _to_py(m["method_counts"]),
    }
    summary = {k: _to_py(v) for k, v in summary.items()}
    path = RESULTS_DIR / "summary.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"✓  Oppsummering lagret → {path.relative_to(ROOT)}")
    return summary


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 62)
    print("  BALL DETECTOR — YTELSESTEST")
    print("  Bachelor 2026, Universitetet i Sørøst-Norge")
    print("=" * 62)
    print(f"\nTid: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Kamera: OAK Series 2, {config.CAMERA_RESOLUTION[0]}×{config.CAMERA_RESOLUTION[1]}")
    print(f"Testlengde: {BENCHMARK_DURATION_S} sekunder")
    print(f"Forventet scene: {EXPECTED_RED_BALLS} røde + {EXPECTED_BLUE_BALLS} blå baller")
    print()
    print("Sørg for at alle baller er synlige for kameraet.")
    input("Trykk ENTER for å starte …")

    # 1. Samle data
    records = run_capture(BENCHMARK_DURATION_S)

    # 2. Lagre CSV
    save_csv(records)

    # 3. Beregn metrics
    m = compute_metrics(records)

    # 4. Generer grafer
    print("\nGenererer grafer …")
    plot_fps(m)
    plot_detections(m)
    plot_confidence(m)
    plot_method_breakdown(m)
    plot_processing_latency(m)

    # 5. Lagre JSON-oppsummering
    summary = save_summary(m)

    # 6. Print terminal-rapport
    print("\n" + "=" * 62)
    print("  TESTRESULTATER")
    print("=" * 62)
    print(f"  Varighet:          {m['duration_s']} s   ({m['n_frames']} frames)")
    print(f"  FPS:               {m['fps_mean']} ± {m['fps_std']}  (min {m['fps_min']}, max {m['fps_max']})")
    print(f"  Latens per frame:  {m['proc_mean_ms']} ms snitt  |  P95 = {m['proc_p95_ms']} ms")
    print()
    print(f"  Rød — snitt/frame: {m['red_mean']:.2f}  |  "
          f"deteksjonsrate: {m['red_det_rate']}%  |  "
          f"conf: {m['conf_red_mean']:.2f}")
    print(f"  Blå — snitt/frame: {m['blue_mean']:.2f}  |  "
          f"deteksjonsrate: {m['blue_det_rate']}%  |  "
          f"conf: {m['conf_blue_mean']:.2f}")
    print()
    print(f"  Deteksjonsmetoder: {m['method_counts']}")
    print("=" * 62)
    print(f"\nAlle resultater lagret i: tests/vision_benchmark/results/")


if __name__ == "__main__":
    main()
