#!/usr/bin/env python3
"""
diagnose_motors.py
==================
Diagnostic tool for the OpenRB-150 + Dynamixel motor chain.

Connects to the OpenRB-150 via USB serial and sends the "diagnose" command
which pings each motor (IDs 1-5) at multiple Dynamixel baud rates
(57600, 115200, 1000000).  Prints a clear, colour-coded report telling the
user exactly which motors respond, at what baud rate, and what to fix.

Usage:
    python diagnose_motors.py [--port /dev/cu.usbmodem101]

Author: Bachelor Project 2026 – Autonomia
"""

import argparse
import json
import sys
import time
from typing import Optional

try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed.  Run:  pip install pyserial")
    sys.exit(1)

# ── Defaults ──────────────────────────────────────────────────────────
DEFAULT_PORT = "/dev/cu.usbmodem101"
USB_BAUD     = 115200          # must match PI_BAUDRATE in the firmware
TIMEOUT      = 8               # seconds – diagnose scans 3 baud rates × 5 motors

# ── ANSI colours (disable on Windows without colorama) ────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

# ── Expected motor descriptions ──────────────────────────────────────
MOTOR_LABELS = {
    1: "Base Pan      (XM430)",
    2: "Shoulder Tilt (XM540)",
    3: "Elbow Tilt    (XM430)",
    4: "Wrist Tilt    (XL430)",
    5: "Claw          (XL430)",
}

# Known Dynamixel model numbers
MODEL_NAMES = {
    1020: "XM430-W350",
    1030: "XM430-W210",
    1120: "XM540-W270",
    1130: "XM540-W150",
    1060: "XL430-W250",
    1090: "XL430-W250-2",
}


def open_serial(port: str) -> serial.Serial:
    """Open serial connection to the OpenRB-150."""
    try:
        ser = serial.Serial(port, USB_BAUD, timeout=TIMEOUT)
        time.sleep(2)  # wait for Arduino reset after DTR toggle
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as exc:
        print(f"{RED}ERROR: Cannot open {port}: {exc}{RESET}")
        print()
        print("Possible fixes:")
        print("  1. Check that the OpenRB-150 is connected via USB-C")
        print("  2. Verify the port name:  ls /dev/cu.usbmodem*")
        print("  3. Close any other program using the port (Arduino IDE Serial Monitor)")
        sys.exit(1)


def drain_boot_messages(ser: serial.Serial) -> list[str]:
    """Read and return any boot/startup messages sitting in the buffer."""
    lines = []
    while True:
        raw = ser.readline()
        if not raw:
            break
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            lines.append(line)
    return lines


def send_diagnose(ser: serial.Serial) -> Optional[dict]:
    """Send the diagnose command and parse the JSON response."""
    cmd = json.dumps({"cmd": "diagnose"}) + "\n"
    ser.write(cmd.encode("utf-8"))
    ser.flush()

    # The diagnose command may take a while (switching baud rates, pinging).
    # Read lines until we get a JSON response starting with '{'.
    deadline = time.time() + TIMEOUT
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        if line.startswith("{"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                print(f"{YELLOW}WARNING: Received malformed JSON: {line}{RESET}")
                continue
        else:
            # Might be an ERR line or boot message
            print(f"  [OpenRB] {line}")

    return None


def print_report(data: dict) -> None:
    """Pretty-print the diagnostic results with actionable advice."""
    motors = data.get("diagnostics", [])
    if not motors:
        print(f"{RED}ERROR: No motor data in response.{RESET}")
        return

    firmware_baud = 115200  # DXL_BAUDRATE in the firmware

    found_count = 0
    mismatch_bauds: list[tuple[int, int]] = []  # (motor_id, actual_baud)

    print()
    print(f"{BOLD}{'='*60}{RESET}")
    print(f"{BOLD}  DYNAMIXEL MOTOR DIAGNOSTIC REPORT{RESET}")
    print(f"{BOLD}{'='*60}{RESET}")
    print()

    for m in motors:
        mid = m["id"]
        label = MOTOR_LABELS.get(mid, f"Motor {mid}")
        found = m.get("found", False)

        if found:
            found_count += 1
            baud = m.get("baud", 0)
            pos  = m.get("position", 0)
            model_num = m.get("model", 0)
            model_name = MODEL_NAMES.get(model_num, f"Unknown({model_num})")

            baud_ok = (baud == firmware_baud)
            baud_color = GREEN if baud_ok else YELLOW

            print(f"  {GREEN}✔{RESET}  ID {mid}  {label}")
            print(f"       Model:    {model_name}")
            print(f"       Position: {pos}")
            print(f"       Baud:     {baud_color}{baud}{RESET}", end="")
            if not baud_ok:
                print(f"  {YELLOW}⚠ MISMATCH (firmware expects {firmware_baud}){RESET}")
                mismatch_bauds.append((mid, baud))
            else:
                print(f"  {GREEN}(matches firmware){RESET}")
            print()
        else:
            print(f"  {RED}✘{RESET}  ID {mid}  {label}")
            print(f"       {RED}NOT FOUND at any baud rate (57600 / 115200 / 1000000){RESET}")
            print()

    # ── Summary ───────────────────────────────────────────────────────
    print(f"{BOLD}{'─'*60}{RESET}")
    print(f"  {BOLD}Summary:{RESET}  {found_count}/{len(motors)} motors detected")
    print(f"{BOLD}{'─'*60}{RESET}")
    print()

    if found_count == 0:
        print(f"{RED}{BOLD}  ⚠  NO MOTORS DETECTED AT ALL{RESET}")
        print()
        print("  Checklist:")
        print("  1. Verify the TTL cable is connected from the OpenRB-150")
        print("     Dynamixel port to the first motor in the daisy chain.")
        print("  2. Confirm external 12V power supply is ON and connected")
        print("     to the OpenRB-150 power jack (not just USB).")
        print("  3. Check each motor has a unique ID (use Dynamixel Wizard 2.0).")
        print("  4. Try connecting a single motor directly to the OpenRB-150")
        print("     (bypass the daisy chain) to isolate wiring issues.")
        print()
        return

    if mismatch_bauds:
        print(f"  {YELLOW}{BOLD}⚠  BAUD RATE MISMATCH DETECTED{RESET}")
        print()
        print(f"  The firmware talks to motors at {BOLD}{firmware_baud}{RESET} baud,")
        print(f"  but these motors are at a different baud rate:")
        print()
        for mid, actual in mismatch_bauds:
            print(f"    Motor ID {mid}: currently at {BOLD}{actual}{RESET} baud")
        print()
        print(f"  {BOLD}How to fix:{RESET}")
        print(f"    Option A – Change motor baud rates to {firmware_baud}:")
        print(f"      Use Dynamixel Wizard 2.0 → connect at {mismatch_bauds[0][1]} baud")
        print(f"      → select each motor → set Baud Rate to {firmware_baud} → save.")
        print()
        print(f"    Option B – Change firmware baud rate to {mismatch_bauds[0][1]}:")
        print(f"      In openrb_bridge.ino, change:")
        print(f"        const uint32_t DXL_BAUDRATE = {firmware_baud};")
        print(f"      to:")
        print(f"        const uint32_t DXL_BAUDRATE = {mismatch_bauds[0][1]};")
        print(f"      Then re-upload the firmware.")
        print()

    if found_count == len(motors) and not mismatch_bauds:
        print(f"  {GREEN}{BOLD}✔  All motors detected and baud rates match!{RESET}")
        print()
        print("  If motors still don't move, check:")
        print("  1. Torque might be disabled – the diagnose command doesn't enable it.")
        print("     Restart the OpenRB-150 to re-run setup() which enables torque.")
        print("  2. Goal positions might be the same as current positions.")
        print("  3. Check for hardware error status (LED blinking on motor).")
        print()

    missing = [m for m in motors if not m.get("found")]
    if missing and found_count > 0:
        print(f"  {YELLOW}Some motors are missing. For the missing motor(s):{RESET}")
        print("  1. Check the daisy-chain cable between the last found motor")
        print("     and the first missing motor.")
        print("  2. Verify the motor ID using Dynamixel Wizard 2.0.")
        print("  3. The motor may be defective – try swapping it with a working one.")
        print()


def main():
    parser = argparse.ArgumentParser(
        description="Diagnose Dynamixel motor connectivity via OpenRB-150"
    )
    parser.add_argument(
        "--port", "-p",
        default=DEFAULT_PORT,
        help=f"Serial port for the OpenRB-150 (default: {DEFAULT_PORT})",
    )
    args = parser.parse_args()

    print(f"{BOLD}Dynamixel Motor Diagnostics{RESET}")
    print(f"{'─'*40}")
    print(f"  Port:  {args.port}")
    print(f"  Baud:  {USB_BAUD} (USB serial)")
    print()

    # ── Connect ───────────────────────────────────────────────────────
    print(f"Connecting to OpenRB-150 on {CYAN}{args.port}{RESET} ...")
    ser = open_serial(args.port)
    print(f"{GREEN}Connected.{RESET}")
    print()

    # ── Drain boot messages ───────────────────────────────────────────
    boot = drain_boot_messages(ser)
    if boot:
        print("Boot messages from OpenRB-150:")
        for line in boot:
            print(f"  {line}")
        print()

    # ── Send diagnose command ─────────────────────────────────────────
    print("Sending diagnose command (scanning 3 baud rates × 5 motors)...")
    print("This may take a few seconds...")
    print()

    data = send_diagnose(ser)
    ser.close()

    if data is None:
        print(f"{RED}ERROR: No response from OpenRB-150.{RESET}")
        print()
        print("Possible causes:")
        print("  1. Firmware not uploaded – upload openrb_bridge.ino via Arduino IDE.")
        print("  2. Firmware doesn't have the 'diagnose' command – re-upload the")
        print("     updated openrb_bridge.ino that includes the diagnose handler.")
        print("  3. Serial communication issue – try unplugging and re-plugging USB.")
        sys.exit(1)

    print_report(data)


if __name__ == "__main__":
    main()
