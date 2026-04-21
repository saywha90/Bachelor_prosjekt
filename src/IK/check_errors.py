#!/usr/bin/env python3
import json
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial is not installed.")
    sys.exit(1)

PORT = "/dev/cu.usbmodem101"
BAUD = 115200

def parse_error_status(status_byte: int) -> list:
    errors = []
    if status_byte & 0x01: errors.append("Input Voltage Error")
    if status_byte & 0x04: errors.append("Overheating Error")
    if status_byte & 0x08: errors.append("Motor Encoder Error")
    if status_byte & 0x10: errors.append("Electrical Shock Error")
    if status_byte & 0x20: errors.append("Overload Error")
    return errors

def main():
    print(f"Connecting to OpenRB-150 on {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(2)  # wait for DTR reset
        ser.reset_input_buffer()
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("Sending 'read_errors' command...")
    cmd = json.dumps({"cmd": "read_errors"}) + "\n"
    ser.write(cmd.encode("utf-8"))
    
    resp_raw = ser.readline().decode("utf-8").strip()
    if not resp_raw:
        print("No response from OpenRB-150. Are you sure you uploaded the new firmware?")
        return
    
    try:
        errors = json.loads(resp_raw)
        print("\n=== HARDWARE ERROR STATUS ===")
        for motor_key, status in errors.items():
            parsed = parse_error_status(status)
            if not parsed:
                print(f"  {motor_key.upper()}: OK (0)")
            else:
                print(f"  {motor_key.upper()}: ERROR ({status}) -> {', '.join(parsed)}")
                
        print("\nNote: Once a motor is in an error state (blinking red), you MUST")
        print("disconnect its 12V power to clear the error. Software commands")
        print("cannot magically reset the physical hardware error flag.")
        
    except json.JSONDecodeError:
        print(f"Failed to parse response: {resp_raw}")

if __name__ == "__main__":
    main()
