# Fordeling av arbeid — Bachelorprosjekt Autonomia 2026

Dette dokumentet beskriver hvilke deler av kodebasen hver gruppemedlem
"eier" og presenterer i bachelorrapporten. Hver person har en
sammenhengende rød tråd som dekker både implementasjon, testing og
designvalg.

| Person | Rolle | Tema |
|--------|-------|------|
| **U** | Kinematikk, hardware & verifisering | IK-solver, 3D-visualisering, FK-tester, touch-/sag-/scan-kalibrering, hardware-valg, motor-diagnostikk, termal-beskyttelse |
| **F** | Firmware & system-integrasjon | OpenRB-bro, hovedløkke, konfigurasjonsarkitektur, simulering, motor-kalibrering |
| **O** | Datasyn (vision) | Kamera, ball-deteksjon, klassifisering, HSV-/homografi-kalibrering |

---

## 👤 U — Den Fysiske Armen: Hardware, Kinematikk, Verifisering & Diagnostikk

**Rød tråd:** *"Hele den fysiske armen — fra motorvalg og link-lengder,
gjennom matematikken som styrer bevegelsen, til verifisering, 3D-visualisering,
termal-beskyttelse og feilsøking."*

### Filer

| Fil | Hva den gjør |
|-----|--------------|
| `src/ik/solver.py` ⭐ | Geometrisk inverse kinematikk for 4-DOF arm. Tar `(x, y, z)` cm → motor-steg. Inneholder dynamisk wrist-pitch loop, sag-kompensasjon, joint limits og reach clamping. |
| `src/simulation/visualizer.py` | Live 3D matplotlib-rendering av armen via forward kinematics. Bruker FK som invers av solveren — fungerer både som demo og sanity-check. |
| `src/calibration/09_touch_calibration.py` | Interaktiv touch-kalibrering av homografi-matrisen. Auto-deteksjon av ballsentre, frame-averaging, N-punkt RANSAC, reprojeksjons-feilrapport. Erstatter linjals-måling. |
| `src/calibration/03_sag.py` | Måler gravitasjons-droop på flere rekkevidde-distanser. Fitter linær + kvadratisk modell og lagrer `sag_calibration.json` (auto-lastet av `ArmIK`). |
| `src/calibration/02c_scan_pose.py` | Kalibrerer SCAN_POSE manuelt med WASD-styring — definerer hvor armen parkerer kameraet for å se hele arbeidsrommet. |
| `tests/test_ik_solver.py` | Unit-tester med FK round-trip, symmetri-bevis (mirror-Y), sweep-validering, dynamic pitch og edge cases. |
| `tests/conftest.py` | Pytest-fixtures for IK-testene (`arm`, `arm_no_sag`, `home_position`, `bin_positions`). |
| `src/ik/sag_calibration.json` | Output fra `03_sag.py` — auto-lastet av solveren. |
| `scripts/manual_tests/ik_virtual_demo.py` | Virtuelt test-rammeverk for `solver.py`. Mater inn fiktive kamera-koordinater, printer JSON-output og flagger mistenkelige hopp mellom nærliggende mål. Ren matematikk-validering uten hardware. |
| `tests/test_main_m3_thermal.py` | Tester M3 termal-beskyttelseslogikken — strømlesing, SCAN_POSE current-limit, torque-relax. Direkte koblet til SCAN_POSE-designet. |
| `src/diagnostics/diagnose_motors.py` | Pinger alle 5 Dynamixel-motorer ved flere baud-rater (57600, 115200, 1M). Identifiserer hvilke motorer som svarer. |
| `src/diagnostics/check_motor_errors.py` | Leser hardware-error-flags fra Dynamixel-registre (overheat, overload, voltage, encoder, electrical shock). Krever 12V power cycle for å nullstilles. |
| `src/diagnostics/stream_debug.py` | Live-stream av motor-data (posisjon, last, temperatur, strøm) for sanntids-feilsøking. |

### Dokumentasjon

| Dokument | Beskrivelse |
|----------|-------------|
| `docs/decisions/002-4dof-geometry.md` ⭐ | **Designvalg:** Hvorfor geometrisk IK fremfor numeriske/ML-tilnærminger. Inneholder ferdig sammenligningstabell mot Jacobian/gradient descent, ML-IK og Denavit–Hartenberg. |
| `docs/decisions/003-fixed-scan-pose.md` | **Designvalg:** Hvorfor fast SCAN_POSE fremfor adaptiv scanning. Direkte koblet til `02c_scan_pose.py`. |
| `docs/decisions/004-touch-calibration-replaces-homography.md` | Støtter `09_touch_calibration.py` — hvorfor touch erstatter linjals-måling. |
| `docs/troubleshooting.md` | Feilsøkingsguide — IK-relaterte problemer (rekkevidde, joint limits, sag), SCAN_POSE-justering, M3 termal-issues. |
| `docs/hardware.md` | Maskinvarespesifikasjoner og hardware-valg — Dynamixel-motorvalg (XM430/XM540/XL430), link-lengder, kabling, hvorfor disse motorene ble valgt. |

### Snakke-temaer

- Lukket-form geometrisk IK (Lov om cosinus + `atan2`)
- Dynamisk wrist-pitch loop som utvider rekkevidden
- Sag-kompensasjon (linær + kvadratisk modell, auto-lastet fra JSON)
- Joint limits + reach clamping (forhindrer maskinvarefeil)
- **FK round-trip-tester** — `_forward_kinematics_xy()` reverserer IK og verifiserer
- **Symmetri-bevis** — `(x, +y, z)` og `(x, −y, z)` gir `m1_pos + m1_neg ≈ 4096`
- **Sweep-validering** — parametrisert over alle bin-posisjoner og koordinater
- 3D-visualisering med Poly3DCollection og ghost trail
- Touch-kalibrering med sub-piksel deteksjon og RANSAC
- **SCAN_POSE-design** — fast pose for wrist-mounted OAK-D, hvorfor fast fremfor adaptiv (ADR-003)
- Feilsøking av IK-relaterte problemer (rekkevidde, joint limits, sag)
- **Hardware-valg** — hvorfor XM540 i skulder (høy stall-torque), XM430 i albue og klo (høyt dreiemoment for grep), XL430 i håndledd (lett/billig der lasten er lav)
- Link-lengder L1/L2/L3 — hvordan de ble målt og hvilken arbeidsromsdekning de gir
- **M3 termal-beskyttelse** — XM430 i albue blir varm i SCAN_POSE (0.47 A kontinuerlig), redusert hold-strøm (300 mA), torque-relax-mekanisme
- **Hardware-error-flags** — overheat, overload, voltage, encoder, electrical shock; krever 12V power cycle å nullstille
- Diagnostiske verktøy — ping over flere baud-rater, error-flag-lesing, live-streaming av motor-data

---

## 👤 F — Firmware & System-Integrasjon

**Rød tråd:** *"Hvordan systemet henger sammen — firmware-broen mellom
Pi og motorer, hovedløkken som orkestrerer alt, og konfigurasjonen som
binder IK til den virkelige verden."*

### Filer

| Fil | Hva den gjør |
|-----|--------------|
| `firmware/openrb_bridge/openrb_bridge.ino` ⭐ | OpenRB-150 firmware (Arduino/C++). USB-bro mellom Raspberry Pi og 5 daisy-chained Dynamixel-motorer. JSON-protokoll over seriell. |
| `src/main.py` ⭐ | Hovedløkken — orkestrerer scan → detect → pick → place. Kobler IK, vision og firmware sammen. Inneholder retry-logikk, grip-verifikasjon og bin-kalibrerings-integrasjon (SORTING/DROPPING-states bruker kalibrerte bin-posisjoner). |
| `src/config/arm.py` | Fysiske konstanter, bin-posisjoner, `SCAN_POSE`, `HOME_POSITION`, link-lengder (L1/L2/L3), grab heights, sag-modell parametere, `MAX_REACH_PITCH`, `compute_grab_height()`, `compute_wrist_correction()`, `load_bin_calibration()`, `get_bin_coords()`, `get_bin_m4_offset()`. Bindeleddet mellom IK og virkelig geometri. |
| `src/simulation/mock_serial.py` | Falsk seriell-port for testing av `main.py` uten fysisk hardware. |
| `src/calibration/02_joints.py` | Kalibrerer null-punkter for hver motor (sign + zero offset). |
| `src/calibration/02b_claw.py` | Kalibrerer klo-åpning og lukket-posisjon. |
| `src/calibration/10_bin_calibration.py` | Interaktiv bin-posisjon-kalibrering med WASD-styring + limp mode. Lagrer kalibrerte bin-koordinater til JSON. |
| `src/calibration/08_pick_test.py` | End-to-end pick-test med en kjent ball-posisjon. |
| `scripts/manual_tests/record_stats.py` | Tar opp ytelses-statistikk (latens, FPS, syklustid) under kjøring for `docs/performance.md`. |

### Dokumentasjon

| Dokument | Beskrivelse |
|----------|-------------|
| `docs/architecture.md` | System-arkitektur: Pi → OpenRB → motorer + Pi → kamera. |
| `docs/pi-setup.md` | Oppsett av Raspberry Pi (OS, drivere, dependencies). |
| `docs/performance.md` | Ytelsesmålinger (latens, FPS, syklustid). |

### Snakke-temaer

- JSON-protokoll over USB-seriell (`{"m1":...,"m2":...}\n` → `OK\n`)
- Dynamixel2Arduino-bibliotek og daisy-chain på Serial1
- Firmware-arkitektur i `openrb_bridge.ino` — kommando-parsing, motor-bus håndtering
- Hovedløkkens tilstandsmaskin (HOME → SCAN → DETECT → PICK → PLACE)
- Konfigurasjonsarkitektur i `arm.py` — bin-koordinater, HOME/SCAN_POSE, distanse-basert grab-height interpolasjon
- Grip-verifikasjon med last-måling
- Retry-logikk og feilhåndtering på system-nivå
- Mock-seriell for hardware-løs testing
- Motor-kalibrering (joints, claw)
- Bin-kalibrering — interaktiv WASD-styring + limp mode for presis plassering av bin-koordinater
- `load_bin_calibration()` / `get_bin_coords()` / `get_bin_m4_offset()` — dynamisk lasting og oppslag av kalibrerte bin-posisjoner
- SORTING/DROPPING-states i `main.py` — bruk av kalibrerte bin-posisjoner for presist ball-avkast
- End-to-end pick-test integrasjon

---

## 👤 O — Datasyn (Vision)

**Rød tråd:** *"Hvordan armen ser — fra kamerapiksler til fargede baller
i koordinater."*

### Filer

| Fil | Hva den gjør |
|-----|--------------|
| `src/vision/camera.py` ⭐ | OAK-D S2 kamera-wrapper (DepthAI). Frame-grabbing, oppløsning, fokal-lengde. |
| `src/vision/detector.py` ⭐ | `SimpleBallDetector` — ensemble av HSV + Hough Circle deteksjon med adaptiv lysjustering, multi-tracking og konfidens-scoring. |
| `src/vision/classifier.py` | Farge-klassifisering (rød/blå/ukjent) med shape + color confidence. |
| `src/ik/vision_bridge.py` | Bro mellom OAK-D og IK-statemaskinen. Konverterer piksel-koordinater til arm-frame cm via homografi. |
| `src/config/vision.py` | Vision-konstanter: kamera-oppløsning, HSV-grenser, ball-radius, konfidens-terskler. |
| `src/calibration/04_hsv_tuner.py` | Interaktiv HSV-tuner med trackbars for å finne fargegrenser. |
| `src/calibration/05_hsv_refine.py` | Forfining av HSV-grenser med statistisk analyse. |
| `src/calibration/06_homography.py` | Manuell homografi-kalibrering (eldre metode, erstattet av 09). |
| `src/calibration/07_vision_offset.py` | Finjustering av kamera-til-skulder offset. |
| `src/calibration/homography_calibration.json` | Kalibrert homografi + height_calibration + wrist_calibration. |
| `src/training/capture_data.py` | Innsamling av treningsdata for klassifikatoren. |
| `src/training/train_classifier.py` | Trening av farge-klassifikator. |
| `src/diagnostics/diagnose_detection.py` | Live-debugging av deteksjonspipelinen. |
| `scripts/manual_tests/enhanced_detector_demo.py` | Demo av deteksjonsforbedringer. |
| `scripts/manual_tests/oak_v3_demo.py` | OAK-D V3 funksjonalitets-demo. |
| `scripts/manual_tests/backend_check.py` | Sjekker DepthAI-backend og kamera-tilkobling før kjøring. |

### Dokumentasjon

| Dokument | Beskrivelse |
|----------|-------------|
| `docs/calibration.md` | Komplett kalibrerings-guide (HSV → homografi → touch). |
| `docs/vision-history.md` | Historikk og evolusjon av vision-pipelinen. |
| `docs/decisions/001-hsv-over-cnn.md` ⭐ | **Designvalg:** Hvorfor HSV-deteksjon fremfor CNN-basert klassifisering. |

### Snakke-temaer

- OAK-D S2 oppsett og DepthAI-pipeline
- HSV-fargesegmentering vs. CNN (ADR-001)
- Ensemble-deteksjon: HSV + Hough Circle med voting
- Adaptiv lysjustering (lighting_level)
- Multi-ball tracking med track_id og persistens
- Sub-piksel ball-sentre via kontur-momenter
- Homografi-transformasjon (piksel → cm)
- Konfidens-scoring (shape_confidence + color_confidence)
- HSV-tuning og forfining

---

## 🤝 Felles eierskap (nevnes kort av alle)

Disse filene berører alle tre områder og kan refereres til i alle
presentasjoner:

| Fil | Kommentar |
|-----|-----------|
| `README.md` | Prosjektoversikt — alle bidrar. |
| `CHANGELOG.md` | Loggfører endringer på tvers av domener. |
| `pyproject.toml` | Pakke-konfigurasjon, pytest-oppsett. |
| `requirements.txt` / `Pipfile` | Dependencies. |
| `docs/fordeling_av_arbeid.md` | Dette dokumentet — fordeling av arbeid mellom U, F og O. |
| `src/calibration/README.md` | Oversikt over alle kalibreringsstegene 02–10 (alle tre eier ulike steg). |
| `scripts/manual_tests/README.md` | Beskriver `manual_tests/`-mappen og demo-scriptene. |

---

## 📈 Fordelingsstatistikk

| Eier | Kode-filer | Dokumenter | Totalt |
|------|-----------:|-----------:|-------:|
| **U** |    13     |      5     | **18** |
| **F** |    9      |      3     | **12** |
| **O** |    16     |      3     | **19** |
| Felles |    5     |      2     | **7**  |
| **Sum** |  **43** |   **13**   | **56** |

> *Antall ADRs (designvalg) per person: U har 3 (002, 003, 004) + hardware-valg, F har 0 dedikerte, O har 1 (001).*

---

## 📊 Visuell oversikt

```
                    ┌──────────────────────┐
                    │    main.py (F)       │
                    │  Hovedløkke          │
                    └──┬────────┬──────┬───┘
                       │        │      │
            ┌──────────▼──┐  ┌──▼─────────┐
            │ vision/ (O) │  │  ik/ (U)   │
            │  - camera   │  │  - solver  │
            │  - detector │◄─┤            │  ◄── config/arm.py (F)
            │  - classif. │  └──┬─────────┘      (parametere)
            └─────────────┘     │
                                │
                    ┌───────────▼────────────┐
                    │  visualizer (U)        │
                    │  3D + Forward Kinem.   │
                    │  + tester (U)          │
                    └────────────────────────┘
                                │
                    ┌───────────▼────────────┐
                    │  openrb_bridge (F)     │
                    │  Arduino firmware      │
                    └───────────┬────────────┘
                                │
                    ┌───────────▼────────────┐
                    │  5× Dynamixel motorer  │  ◄── diagnostics/ (U)
                    │  (XM430/XM540/XL430)   │      hardware.md (U)
                    │   ↑ termal-beskyttelse │      termal-tester (U)
                    └────────────────────────┘
```

### Domeneansvar i ett blikk

| Lag | Eier |
|-----|------|
| Hardware-valg (motorer, link-lengder) | **U** |
| Firmware (Arduino C++) | **F** |
| Konfigurasjon (arm.py, vision.py) | F / O |
| IK-matematikk (solver, FK, tester, visualizer) | **U** |
| Vision-pipeline (kamera, deteksjon, klassifisering) | **O** |
| System-integrasjon (main.py, hovedløkke) | **F** |
| Kalibrering — IK-relatert (sag, scan-pose, touch) | **U** |
| Kalibrering — motor (joints, claw, bin) | **F** |
| Kalibrering — vision (HSV, homografi-offset) | **O** |
| Motor-diagnostikk & termal-beskyttelse | **U** |
| Feilsøking & troubleshooting | **U** |

---

*Sist oppdatert: 2026 — Bachelor Project Autonomia*
