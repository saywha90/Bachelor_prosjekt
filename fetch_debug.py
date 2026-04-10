#!/usr/bin/env python3
"""
fetch_debug.py  (kjøres på Mac)
================================

1. SSH-er inn på Pi og starter capture_debug_frames.py
2. Venter til skriptet er ferdig
3. Henter alle frames + JSON med scp
4. Åpner alle annoterte frames + masker i et OpenCV-vindu slik at
   du kan bla gjennom dem med piltastene

Bruk:
    python fetch_debug.py [--frames 30] [--host robotpi.local]

Krav (Mac):
    pip install paramiko   (eller: brew install openssh er nok for scp)
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np


PI_USER    = "pi"
PI_SCRIPT  = "~/Bachelor_prosjekt/src/vision/capture_debug_frames.py"
PI_OUT_DIR = "/tmp/debug_out"
LOCAL_OUT  = Path(__file__).parent / "debug_out"


def run_ssh(host: str, cmd: str) -> int:
    """Kjør kommando på Pi via ssh, vis output løpende."""
    full = f"ssh {PI_USER}@{host} '{cmd}'"
    print(f"  $ {full}")
    return subprocess.call(full, shell=True)


def fetch_results(host: str) -> int:
    """hent debug_out-mappen fra Pi til lokal mappe."""
    # Slett lokalt først (unngår nested debug_out/debug_out)
    if LOCAL_OUT.exists():
        subprocess.call(f'rm -rf "{LOCAL_OUT}"', shell=True)
    LOCAL_OUT.parent.mkdir(parents=True, exist_ok=True)
    # Kopi UTEN trailing slash på kilde: scp kopierer selve mappen inn i target-mappe.
    # Resultat: LOCAL_OUT.parent/debug_out/ = LOCAL_OUT/
    cmd = (
        f'scp -r {PI_USER}@{host}:{PI_OUT_DIR} "{LOCAL_OUT.parent}/"'
    )
    print(f"  $ {cmd}")
    return subprocess.call(cmd, shell=True)


def browse_results(out_dir: Path) -> None:
    """Vis frames + masker med OpenCV — piltaster blar, 'q' avslutter."""    # Fallback: scp kan ha lagt filene i en undermappe med samme navn
    nested = out_dir / "debug_out"
    if not any(out_dir.glob("frame_*.jpg")) and nested.exists():
        out_dir = nested
    frame_files = sorted(out_dir.glob("frame_*.jpg"))
    mask_files  = sorted(out_dir.glob("mask_*.jpg"))

    if not frame_files:
        print("Ingen frames funnet i", out_dir)
        return

    # Last JSON
    json_path = out_dir / "results.json"
    results   = []
    if json_path.exists():
        with open(json_path, encoding="utf-8") as f:
            results = json.load(f)

    # Lag en oppsummering
    n_total     = len(results)
    frames_red  = sum(1 for r in results if any(b["color"] == "red"  for b in r["balls"]))
    frames_blue = sum(1 for r in results if any(b["color"] == "blue" for b in r["balls"]))
    all_confs   = [b["confidence"] for r in results for b in r["balls"]]

    print()
    print("=" * 55)
    print(f"FRAMES:           {n_total}")
    print(f"Frames m/ rød:    {frames_red}  ({frames_red/n_total*100:.0f}%)" if n_total else "")
    print(f"Frames m/ blå:    {frames_blue} ({frames_blue/n_total*100:.0f}%)" if n_total else "")
    if all_confs:
        print(f"Snitt confidence: {sum(all_confs)/len(all_confs)*100:.1f}%")
        print(f"Min  confidence:  {min(all_confs)*100:.1f}%")
        print(f"Maks confidence:  {max(all_confs)*100:.1f}%")

    # Finn frames uten deteksjon
    missed = [r["frame"] for r in results if not r["balls"]]
    if missed:
        print(f"Frames UTEN deteksjon ({len(missed)}): {missed[:20]}{'...' if len(missed)>20 else ''}")
    else:
        print("Alle frames hadde minst én deteksjon!")
    print("=" * 55)
    print()
    print("Blas gjennom frames: ← → piltaster  |  'm' toggle maske  |  'q' avslutt")

    WIN = "Debug Frames"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 820)

    idx       = 0
    show_mask = False

    while True:
        frame_img = cv2.imread(str(frame_files[idx]))
        mask_img  = cv2.imread(str(mask_files[idx])) if idx < len(mask_files) else None

        # Legg på frame-nummer + flags
        display = (mask_img if (show_mask and mask_img is not None) else frame_img)
        if display is None:
            display = np.zeros((400, 640, 3), dtype=np.uint8)

        label = (
            f"[{idx+1}/{len(frame_files)}]  "
            f"{'MASKE' if show_mask else 'ANNOTERT'}  "
            f"(← → bla, m=maske, q=avslutt)"
        )
        cv2.putText(display, label, (10, display.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 3)
        cv2.putText(display, label, (10, display.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # JSON-info for dette framenummeret
        if idx < len(results):
            rd = results[idx]
            for bi, b in enumerate(rd["balls"]):
                clr  = b["color"].upper()
                conf = b["confidence"] * 100
                info = f'{clr} {conf:.0f}%  d={b["distance_cm"]}cm  ID={b["track_id"]}'
                yy   = 30 + bi * 26
                cv2.putText(display, info, (display.shape[1]-370, yy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0,0,0), 3)
                cv2.putText(display, info, (display.shape[1]-370, yy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0,255,80), 1)

        cv2.imshow(WIN, display)
        cv2.waitKey(1)          # pump event loop (nødvendig på macOS)
        raw = cv2.waitKey(0)
        key = raw & 0xFF

        if key in (ord("q"), 27):
            break
        elif key in (ord("a"), 81, 2):    # a / ← Linux / ← macOS
            idx = max(0, idx - 1)
        elif key in (ord("d"), 83, 3):    # d / → Linux / → macOS
            idx = min(len(frame_files) - 1, idx + 1)
        elif key == ord("m"):
            show_mask = not show_mask

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int,  default=30,              help="Antall frames")
    parser.add_argument("--host",   type=str,  default="robotpi.local", help="Pi hostname/IP")
    parser.add_argument("--browse-only", action="store_true",
                        help="Ikke koble til Pi — bla gjennom allerede hentede frames")
    args = parser.parse_args()

    if not args.browse_only:
        print("=" * 55)
        print("STEG 1: Fjern gamle resultater på Pi")
        run_ssh(args.host, f"rm -rf {PI_OUT_DIR}")

        print()
        print(f"STEG 2: Kjør detektor ({args.frames} frames) på Pi")
        venv = "source ~/Bachelor_prosjekt/.venv/bin/activate"
        cmd  = (
            f"{venv} && "
            f"cd ~/Bachelor_prosjekt && "
            f"python {PI_SCRIPT} --frames {args.frames} --out {PI_OUT_DIR}"
        )
        rc = run_ssh(args.host, cmd)
        if rc != 0:
            print(f"FEIL: skript avsluttet med kode {rc}")
            sys.exit(rc)

        print()
        print("STEG 3: Hent resultater til Mac")
        # Slett lokalt
        if LOCAL_OUT.exists():
            subprocess.call(f"rm -rf {LOCAL_OUT}", shell=True)
        fetch_results(args.host)

    print()
    print("STEG 4: Vis frames")
    browse_results(LOCAL_OUT)


if __name__ == "__main__":
    main()
