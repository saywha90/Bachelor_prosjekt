# Raspberry Pi 5 — Setup Guide

Step-by-step instructions to go from a fresh Raspberry Pi 5 to running `main.py`.

## 1. Flash the OS

1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
2. Choose **Raspberry Pi OS (64-bit)** — Bookworm or later.
3. Select your SD card (32 GB+ recommended, Class 10 / U1 minimum).
4. Click the **gear icon** (⚙️) to pre-configure:
   - Enable **SSH**
   - Set **username** and **password**
   - Configure **Wi-Fi** (SSID + password)
   - Set **locale** and **timezone**
5. Flash and insert the SD card into the Pi.

## 2. First Boot

1. Power on the Pi with the 5 V PSU output (USB-C).
2. Find the Pi on your network (e.g., `ping raspberrypi.local` or check your router).
3. SSH in:
   ```bash
   ssh <username>@raspberrypi.local
   ```

## 3. System Update

```bash
sudo apt update && sudo apt upgrade -y
sudo reboot
```

## 4. Install System Dependencies

```bash
sudo apt install -y python3-pip python3-venv git libopencv-dev
```

## 5. DepthAI / OAK-D Udev Rules

The OAK-D S2 camera uses a Movidius VPU. Add udev rules so it can be accessed without root:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and re-plug the OAK-D USB cable after this step.

## 6. Serial Access (OpenRB-150)

The OpenRB-150 motor controller appears as `/dev/ttyACM0`. Your user must be in the `dialout` group to access it:

```bash
sudo usermod -aG dialout $USER
```

**Reboot** for the group change to take effect:

```bash
sudo reboot
```

After reboot, verify:

```bash
groups  # should list "dialout"
ls -l /dev/ttyACM0  # should exist when OpenRB-150 is connected
```

## 7. Clone and Install the Project

```bash
git clone https://github.com/<your-org>/Bachelor_prosjekt.git
cd Bachelor_prosjekt
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

> **Note:** The `pyproject.toml` declares all Python dependencies. If you also need to install from `requirements.txt`:
> ```bash
> pip install -r requirements.txt
> ```

## 8. Verify Hardware Connections

With the OpenRB-150 and OAK-D S2 both plugged in:

```bash
# Check serial port
ls /dev/ttyACM*

# Check OAK-D
python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
```

## 9. Run the Pipeline

```bash
source .venv/bin/activate
python3 src/main.py
```

See [docs/calibration.md](calibration.md) for the full calibration sequence before first use.

## 10. (Optional) Auto-Start on Boot with systemd

Create a systemd service so `main.py` runs automatically:

```bash
sudo tee /etc/systemd/system/ball-sorter.service << 'EOF'
[Unit]
Description=Ball Sorter – main vision + pick pipeline
After=network.target

[Service]
Type=simple
User=<username>
WorkingDirectory=/home/<username>/Bachelor_prosjekt
ExecStart=/home/<username>/Bachelor_prosjekt/.venv/bin/python src/main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable ball-sorter.service
sudo systemctl start ball-sorter.service
```

Check status:

```bash
sudo systemctl status ball-sorter.service
journalctl -u ball-sorter.service -f
```

---

## Troubleshooting

For Pi-specific issues, see [docs/troubleshooting.md](troubleshooting.md).
