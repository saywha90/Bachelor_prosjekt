"""Regenererer summary.json fra allerede lagret raw_data.csv."""
import sys, csv, json
from pathlib import Path
from datetime import datetime
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[2] / "src"))
import config

RESULTS_DIR = Path(__file__).parent / "results"
records = list(csv.DictReader(open(RESULTS_DIR / "raw_data.csv")))

frames_all = {}
for r in records:
    fi = int(r["frame"])
    if fi not in frames_all:
        frames_all[fi] = dict(fps=int(r["fps"]), proc_ms=float(r["proc_ms"]),
                              elapsed_s=float(r["elapsed_s"]), red=0, blue=0,
                              cr=[], cb=[])
    if r["color"] == "red":
        frames_all[fi]["red"] += 1
        if r["confidence"]:
            frames_all[fi]["cr"].append(float(r["confidence"]))
    elif r["color"] == "blue":
        frames_all[fi]["blue"] += 1
        if r["confidence"]:
            frames_all[fi]["cb"].append(float(r["confidence"]))

fl = sorted(frames_all.values(), key=lambda x: x["elapsed_s"])
fps_s  = [f["fps"]       for f in fl]
proc_s = [f["proc_ms"]   for f in fl]
red_s  = [f["red"]       for f in fl]
blue_s = [f["blue"]      for f in fl]
time_s = [f["elapsed_s"] for f in fl]
cr     = [c for f in fl for c in f["cr"]]
cb_l   = [c for f in fl for c in f["cb"]]
mc = {}
for r in records:
    mc[r["method"]] = mc.get(r["method"], 0) + 1

s = {
    "test_timestamp":          datetime.now().isoformat(),
    "camera":                  "Luxonis OAK Series 2 (IMX378)",
    "resolution":              f"{config.CAMERA_RESOLUTION[0]}x{config.CAMERA_RESOLUTION[1]}",
    "usb_connection":          "USB 2.0 via Dell-adapter",
    "duration_s":              round(float(max(time_s)), 2),
    "n_frames":                len(fl),
    "fps_mean":                round(float(np.mean(fps_s)), 1),
    "fps_min":                 round(float(np.min(fps_s)),  1),
    "fps_max":                 round(float(np.max(fps_s)),  1),
    "fps_std":                 round(float(np.std(fps_s)),  2),
    "latency_mean_ms":         round(float(np.mean(proc_s)), 2),
    "latency_p95_ms":          round(float(np.percentile(proc_s, 95)), 2),
    "expected_red_balls":      2,
    "expected_blue_balls":     2,
    "avg_detected_red":        round(float(np.mean(red_s)),  3),
    "avg_detected_blue":       round(float(np.mean(blue_s)), 3),
    "detection_rate_red_pct":  round(float(np.mean([1 if r > 0 else 0 for r in red_s])) * 100, 1),
    "detection_rate_blue_pct": round(float(np.mean([1 if b > 0 else 0 for b in blue_s])) * 100, 1),
    "confidence_mean_red":     round(float(np.mean(cr))   if cr   else 0.0, 3),
    "confidence_mean_blue":    round(float(np.mean(cb_l)) if cb_l else 0.0, 3),
    "method_counts":           {k: int(v) for k, v in mc.items()},
}
json.dump(s, open(RESULTS_DIR / "summary.json", "w"), indent=2, ensure_ascii=False)

print("=== TESTRESULTATER ===")
for k, v in s.items():
    print(f"  {k}: {v}")
print("\nsummary.json lagret OK")
