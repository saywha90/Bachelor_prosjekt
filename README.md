# Bachelor_prosjekt - Autonomia Robotarm

Dette repoet inneholder kontrollsystemet for robotarmen utviklet av "Autonomia" (Bachelor 2026).
Systemet er designet for å være modulært, og støtter både 3-akse og 6-akse konfigurasjoner.

## Systemarkitektur

*   **Høynivå (Raspberry Pi):** Python-kode som håndterer kinematikk (IK), brukerinput og seriell kommunikasjon.
*   **Lavnivå (Arduino Mega):** C++ firmware basert på FreeRTOS som styrer servoer og sikrer sanntidsytelse.

## Struktur

```
.
├── firmware/
│   └── motor_controller.ino  # Arduino-kode (FreeRTOS)
├── src/
│   ├── config.py             # Systemkonfigurasjon (Antall akser, dimensjoner)
│   ├── kinematics.py         # Kinematikk-løser (IK/FK)
│   ├── comms_manager.py      # Seriell kommunikasjon
│   ├── main_rpi.py           # Hovedprogram (CLI)
│   └── requirements.txt      # Python-avhengigheter
└── README.md
```

## Komme i gang

### 1. Python (Raspberry Pi / PC)

Installer avhengigheter:
```bash
pip install -r src/requirements.txt
```

Kjør programmet (standard er Mock Mode for testing uten hardware):
```bash
python3 src/main_rpi.py
```

### 2. Arduino

1.  Åpne `firmware/motor_controller.ino` i Arduino IDE.
2.  Installer bibliotekene **FreeRTOS** (av Richard Barry) og **Servo**.
3.  Last opp til Arduino Mega.

### Konfigurasjon

For å endre fra 3 til 6 akser, eller endre fysiske mål på armen:
1.  Rediger `NUM_JOINTS` og `LINK_LENGTHS` i `src/config.py`.
2.  Rediger `NUM_JOINTS` i `firmware/motor_controller.ino`.

## Test Status
Systemet er testet i simuleringsmodus (Mock Mode).
- [x] Invers Kinematikk (3-akse geometrisk)
- [x] Sjekk av rekkevidde (Range check)
- [x] Pakkegenerering for seriell protokoll
