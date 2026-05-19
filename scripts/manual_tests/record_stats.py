"""
record_stats.py
====================

Kjorer balldeteksjon mot OAK Series 2, samler statistikk per frame,
og eksporterer rapport-klare diagrammer som PNG-filer.

Bruk:
    python scripts/manual_tests/record_stats.py

Trykk 'q' for a avslutte og generere diagrammer.
Diagrammer lagres i: reports/diagrams/
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

import cv2
import sys
import time
import json
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")  # Ingen GUI-vindu for matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.ndimage import uniform_filter1d

from vision.camera import OAKCamera
from vision.detector import SimpleBallDetector, BallColor
from config import vision as config

# --- Hvor diagrammer lagres ---
OUTPUT_DIR = Path("reports/diagrams")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")


def run_test() -> dict:
    """Kjorer live deteksjon og samler radata."""
    detector = SimpleBallDetector(
        min_radius=config.BALL_MIN_RADIUS,
        max_radius=config.BALL_MAX_RADIUS,
        confidence_threshold=config.BALL_CONFIDENCE_THRESHOLD,
        enable_adaptive_lighting=True,
    )

    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)
    if not cam.open():
        print("FEIL: Kunne ikke apne OAK-kameraet.")
        sys.exit(1)

    print("=" * 60)
    print("OPPTAK STARTET")
    print("Hold rod eller bla ball foran kameraet.")
    print("Trykk 'q' for a avslutte og generere rapport.")
    print("=" * 60)

    records = []
    fps_log = []
    start_time = time.time()
    fps_time = time.time()
    fps_frames = 0
    current_fps = 0.0
    frame_idx = 0

    while cam.isOpened():
        ret, frame = cam.read()
        if not ret:
            break

        frame_idx += 1
        fps_frames += 1
        elapsed = time.time() - start_time

        if time.time() - fps_time >= 0.5:
            current_fps = fps_frames / (time.time() - fps_time)
            fps_log.append((elapsed, current_fps))
            fps_frames = 0
            fps_time = time.time()

        balls, _ = detector.detect_balls(frame)

        for ball in balls:
            records.append({
                "time": elapsed,
                "frame": frame_idx,
                "fps": current_fps,
                "color": ball.color.value,
                "confidence": ball.confidence,
                "method": ball.detection_method,
                "radius": ball.radius,
                "cx": ball.center[0],
                "cy": ball.center[1],
            })

        overlay = {
            "FPS": f"{current_fps:.1f}",
            "Frame": frame_idx,
            "Funn": len(balls),
        }
        output = detector.draw_detections(frame, balls, show_info=True, overlay=overlay)
        cv2.putText(output, "OPPTAK PAGAR - trykk Q",
                    (10, output.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 255), 2)
        cv2.imshow("Opptak - OAK Ball Detector", output)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.release()
    cv2.destroyAllWindows()

    total_time = time.time() - start_time
    print(f"\nOpptak ferdig: {frame_idx} frames pa {total_time:.1f}s  ({frame_idx/max(total_time,0.001):.1f} FPS snitt)")
    print(f"Totalt deteksjonshendelser: {len(records)}")

    return {
        "records": records,
        "fps_log": fps_log,
        "total_frames": frame_idx,
        "total_time": total_time,
        "detector_stats": detector.stats,
    }


def generate_report(data: dict):
    """Genererer og lagrer rapport-diagrammer fra radata."""

    records = data["records"]
    fps_log = data["fps_log"]
    total_frames = data["total_frames"]
    total_time = data["total_time"]
    det_stats = data["detector_stats"]

    red_records  = [r for r in records if r["color"] == "red"]
    blue_records = [r for r in records if r["color"] == "blue"]

    # --- FIGUR 1: Stor oversiktsfigur ---
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(
        f"Balldeteksjon - OAK Series 2  |  {TIMESTAMP}\n"
        f"{total_frames} frames  .  {total_time:.1f}s  .  {total_frames/max(total_time,0.001):.1f} FPS snitt",
        fontsize=13, fontweight="bold", y=0.98
    )

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

    # A: Confidence over tid
    ax_conf = fig.add_subplot(gs[0, :2])
    if red_records:
        t_r = np.array([r["time"] for r in red_records])
        c_r = np.array([r["confidence"] for r in red_records])
        ax_conf.scatter(t_r, c_r, color="#e74c3c", s=18, alpha=0.7, label="Rod ball", zorder=3)
        if len(c_r) >= 5:
            ax_conf.plot(t_r, uniform_filter1d(c_r, size=5),
                         color="#c0392b", lw=1.5, linestyle="--", zorder=4)
    if blue_records:
        t_b = np.array([r["time"] for r in blue_records])
        c_b = np.array([r["confidence"] for r in blue_records])
        ax_conf.scatter(t_b, c_b, color="#2980b9", s=18, alpha=0.7, label="Bla ball", zorder=3)
        if len(c_b) >= 5:
            ax_conf.plot(t_b, uniform_filter1d(c_b, size=5),
                         color="#1a5276", lw=1.5, linestyle="--", zorder=4)
    ax_conf.axhline(config.BALL_CONFIDENCE_THRESHOLD, color="gray",
                    linestyle=":", lw=1.2, label=f"Terskel ({config.BALL_CONFIDENCE_THRESHOLD})")
    ax_conf.set_xlabel("Tid (s)")
    ax_conf.set_ylabel("Confidence Score")
    ax_conf.set_title("Confidence Score over tid")
    ax_conf.set_ylim(0, 1.05)
    ax_conf.legend(fontsize=9)
    ax_conf.grid(True, alpha=0.3)

    # B: Confidence histogram
    ax_hist = fig.add_subplot(gs[0, 2])
    bins = np.linspace(0, 1, 20)
    if red_records:
        ax_hist.hist([r["confidence"] for r in red_records],
                     bins=bins, color="#e74c3c", alpha=0.7, label="Rod")
    if blue_records:
        ax_hist.hist([r["confidence"] for r in blue_records],
                     bins=bins, color="#2980b9", alpha=0.7, label="Bla")
    ax_hist.axvline(config.BALL_CONFIDENCE_THRESHOLD, color="gray", linestyle=":", lw=1.2)
    ax_hist.set_xlabel("Confidence Score")
    ax_hist.set_ylabel("Antall deteksjoner")
    ax_hist.set_title("Confidence-fordeling")
    ax_hist.legend(fontsize=9)
    ax_hist.grid(True, alpha=0.3)

    # C: FPS over tid
    ax_fps = fig.add_subplot(gs[1, :2])
    if fps_log:
        t_fps = [x[0] for x in fps_log]
        v_fps = [x[1] for x in fps_log]
        ax_fps.plot(t_fps, v_fps, color="#27ae60", lw=1.8, zorder=3)
        ax_fps.fill_between(t_fps, v_fps, alpha=0.15, color="#27ae60")
        ax_fps.axhline(np.mean(v_fps), color="#1e8449", linestyle="--",
                       lw=1.2, label=f"Snitt {np.mean(v_fps):.1f} FPS")
        ax_fps.legend(fontsize=9)
    ax_fps.set_xlabel("Tid (s)")
    ax_fps.set_ylabel("FPS")
    ax_fps.set_title("Kamera-FPS over tid")
    ax_fps.set_ylim(bottom=0)
    ax_fps.grid(True, alpha=0.3)

    # D: Deteksjonsmetode
    ax_method = fig.add_subplot(gs[1, 2])
    method_counts = {
        "HSV":      det_stats.get("hsv_detections", 0),
        "Hough":    det_stats.get("hough_detections", 0),
        "Ensemble": det_stats.get("ensemble_detections", 0),
    }
    colors_m = ["#8e44ad", "#d35400", "#16a085"]
    bars = ax_method.bar(method_counts.keys(), method_counts.values(),
                         color=colors_m, edgecolor="white", linewidth=0.8)
    for bar, val in zip(bars, method_counts.values()):
        ax_method.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                       str(val), ha="center", va="bottom", fontsize=9)
    ax_method.set_ylabel("Antall deteksjoner")
    ax_method.set_title("Deteksjonsmetode")
    ax_method.grid(True, axis="y", alpha=0.3)

    out_overview = OUTPUT_DIR / f"rapport_oversikt_{TIMESTAMP}.png"
    fig.savefig(out_overview, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"OK Oversiktsfigur lagret: {out_overview}")

    # --- FIGUR 2: Statistikktabell ---
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.axis("off")

    def conf_stats(recs):
        if not recs:
            return "-", "-", "-", "-"
        c = [r["confidence"] for r in recs]
        return f"{np.mean(c):.3f}", f"{np.std(c):.3f}", f"{np.min(c):.3f}", f"{np.max(c):.3f}"

    r_mean, r_std, r_min, r_max = conf_stats(red_records)
    b_mean, b_std, b_min, b_max = conf_stats(blue_records)

    table_data = [
        ["", "Rod ball", "Bla ball", "Totalt"],
        ["Antall deteksjoner", str(len(red_records)), str(len(blue_records)), str(len(records))],
        ["Gjennomsnitt confidence", r_mean, b_mean, "-"],
        ["Std.avvik confidence", r_std, b_std, "-"],
        ["Min confidence", r_min, b_min, "-"],
        ["Maks confidence", r_max, b_max, "-"],
        ["Antall frames", "-", "-", str(total_frames)],
        ["Testtid (s)", "-", "-", f"{total_time:.1f}"],
        ["Snitt FPS", "-", "-", f"{total_frames/max(total_time,0.001):.1f}"],
        ["HSV-deteksjoner", "-", "-", str(det_stats.get("hsv_detections", 0))],
        ["Hough-deteksjoner", "-", "-", str(det_stats.get("hough_detections", 0))],
        ["Ensemble-deteksjoner", "-", "-", str(det_stats.get("ensemble_detections", 0))],
    ]

    tbl = ax2.table(cellText=table_data[1:], colLabels=table_data[0],
                    cellLoc="center", loc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 1.6)
    for (r, c), cell in tbl.get_celld().items():
        if r == 0:
            cell.set_facecolor("#2c3e50")
            cell.set_text_props(color="white", fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#ecf0f1")
        else:
            cell.set_facecolor("white")

    ax2.set_title(f"Statistikkoppsummering - {TIMESTAMP}",
                  fontsize=12, fontweight="bold", pad=15)

    out_table = OUTPUT_DIR / f"rapport_tabell_{TIMESTAMP}.png"
    fig2.savefig(out_table, dpi=200, bbox_inches="tight")
    plt.close(fig2)
    print(f"OK Statistikktabell lagret:  {out_table}")

    # --- JSON radata ---
    out_json = OUTPUT_DIR / f"radata_{TIMESTAMP}.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"OK Radata lagret:            {out_json}")

    print()
    print("=" * 60)
    print(f"Alle filer lagret i: {OUTPUT_DIR.resolve()}")
    print("=" * 60)


if __name__ == "__main__":
    data = run_test()
    generate_report(data)
