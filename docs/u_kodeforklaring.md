# Kodeforklaring for U — kinematikk, hardware, simulering og verifisering

Dokumentet forklarer koden som er tilordnet **U** i arbeidsfordelingen i [docs/fordeling_av_arbeid.md](fordeling_av_arbeid.md:16). Området handler om den fysiske robotarmen: hvordan motorposisjoner beregnes, hvordan bevegelsene verifiseres, og hvilke støtteverktøy som finnes for simulering, kalibrering og diagnostikk.

## Overordnet ansvar

Dokumentet dekker hovedsakelig fire deler av systemet:

1. **Inverse kinematikk og forward kinematics** i [src/ik/solver.py](../src/ik/solver.py:37), der ønsket posisjon i centimeter omregnes til Dynamixel-motorsteg og kobler mål i rommet til motorstyring.
2. **Simulering og visualisering** i [src/simulation/visualizer.py](../src/simulation/visualizer.py:1), [src/simulation/mock_serial.py](../src/simulation/mock_serial.py:1), [src/simulation/route_demo.py](../src/simulation/route_demo.py:1) og [src/simulation/bin_safety.py](../src/simulation/bin_safety.py:1), som støtte for å se ruter før fysisk kjøring.
3. **Kalibrering av fysisk arm** gjennom [src/calibration/03_sag.py](../src/calibration/03_sag.py:1), [src/calibration/02c_scan_pose.py](../src/calibration/02c_scan_pose.py:1), [src/calibration/09_touch_calibration.py](../src/calibration/09_touch_calibration.py:1) og [src/calibration/02b_claw_grip_test.py](../src/calibration/02b_claw_grip_test.py:1), som tilpasser modellen til den ekte armen.
4. **Verifisering og feilsøking** gjennom tester i [tests/test_ik_solver.py](../tests/test_ik_solver.py:1), [tests/test_simulation_route_demo.py](../tests/test_simulation_route_demo.py), [tests/test_adaptive_grip.py](../tests/test_adaptive_grip.py), [tests/test_touch_calibration_safety.py](../tests/test_touch_calibration_safety.py) og diagnostikk i [src/diagnostics/diagnose_motors.py](../src/diagnostics/diagnose_motors.py:1), [src/diagnostics/check_motor_errors.py](../src/diagnostics/check_motor_errors.py:1) og [src/diagnostics/stream_debug.py](../src/diagnostics/stream_debug.py), som kontrollerer IK, ruter, grep, touch-kalibrering og motorstatus.

Den røde tråden i U sin forklaring er koblingen mellom robotarmens fysiske geometri og programvaren. Hovedvekten bør ligge på IK, touch-kalibrering og adaptivt grep. Simulering, tester og diagnostikk omtales kort som støtteverktøy.

---

## Inverse kinematikk: [src/ik/solver.py](../src/ik/solver.py:1)

Den viktigste filen i U sitt område er [src/ik/solver.py](../src/ik/solver.py:1). Den definerer klassen [`ArmIK`](../src/ik/solver.py:37), som beregner motorposisjonene for en 4-DOF robotarm med klo. Input er en ønsket posisjon i arbeidsrommet, for eksempel `x`, `y` og `z` i centimeter, og output er et dictionary med motorverdier `m1` til `m5`.

### Formål

Formålet med [`ArmIK`](../src/ik/solver.py:37) er å oversette en fysisk posisjon til kommandoer som OpenRB-firmwaren kan sende videre til Dynamixel-motorene. Klassen tar hensyn til:

- link-lengdene [`L1`](../src/ik/solver.py:48), [`L2`](../src/ik/solver.py:49) og [`L3`](../src/ik/solver.py:50), fordi geometrien bestemmer hvilke mål som kan nås
- motorenes stegsystem, der 0–4095 tilsvarer 360 grader, fordi motorene ikke styres direkte i centimeter eller grader
- skulderhøyde over bordet via [`shoulder_height`](../src/ik/solver.py:79), fordi z-beregningen må starte fra riktig fysisk høyde
- gravitasjonsdropp gjennom sag-kompensasjon, fordi en utstrakt arm synker litt i praksis
- trygge motorgrenser gjennom [`JOINT_LIMITS`](../src/ik/solver.py:97), fordi motorene må holdes innenfor trygt område
- forskjellen mellom vanlig, tolerant løsning og streng produksjonsvalidering, fordi testing og produksjon trenger ulik feilhåndtering

### Vanlig IK-løsning

Hovedmetoden er [`solve()`](../src/ik/solver.py:329). Den gjør følgende:

1. Leser inn ønsket målposisjon i centimeter, fordi IK-løseren må vite hvor kloen skal ende i fysisk rom.
2. Hindrer at kloen går under minimumshøyden [`Z_MIN`](../src/ik/solver.py:84), fordi roboten ellers kan treffe bordet eller presse kloen ned i underlaget.
3. Beregner horisontal rekkevidde med `sqrt(x² + y²)`, fordi skulder- og albueberegningen trenger avstanden fra basen til målet.
4. Legger til sag-kompensasjon dersom dette ikke er deaktivert, fordi den virkelige armen synker litt når den strekkes ut.
5. Bruker en dynamisk pitch-loop der håndleddet starter rett ned og gradvis vinkles fremover hvis målet er langt unna, fordi håndleddsposisjonen kan gi ekstra rekkevidde uten å endre armens grunngeometri.
6. Bruker cosinussetningen for å beregne skulder- og albuevinkel, fordi armleddene danner en trekant med kjente segmentlengder og kjent avstand til målet.
7. Konverterer vinkelverdiene til Dynamixel-steg, fordi firmwaren og motorene styres med stegbaserte posisjonskommandoer.
8. Sjekker at alle motorverdier ligger innenfor [`JOINT_LIMITS`](../src/ik/solver.py:97), fordi løsningen må stoppes før den kan gi farlige eller mekanisk umulige bevegelser.

Dette betyr at [`solve()`](../src/ik/solver.py:329) ikke bare er ren matematikk, men også inneholder praktiske sikkerhetstiltak for den fysiske roboten. Dersom et mål er litt for langt unna, kan metoden skalere målet inn i rekkevidde i stedet for å krasje. Hvis `strict=True` brukes, vil den heller stoppe med feil.

### Streng IK for produksjonsruter

Metoden [`solve_strict()`](../src/ik/solver.py:554) er laget for produksjonsruter der roboten ikke skal gjette eller klampe seg frem til en løsning. Den brukes når posisjoner allerede er kalibrert og skal være trygge på forhånd. Forskjellen fra [`solve()`](../src/ik/solver.py:329) er at [`solve_strict()`](../src/ik/solver.py:554):

- krever at hensikten er eksplisitt, for eksempel `pickup`, `carry` eller `rear_place`, fordi samme koordinat kan ha ulike sikkerhetskrav
- avviser mål under gulvgrensen i stedet for å klampe dem, fordi produksjonsruter ikke skal skjule farlige mål
- avviser sag-kompensasjon utenfor trygt område, fordi kompensasjonen bare er trygg der den er kalibrert
- avviser brudd på leddgrenser, fordi motorene ikke skal presses mekanisk
- returnerer både `commands` og en `validation`-struktur med forklaring på løsningen, fordi kallende kode trenger både kommandoer og begrunnelse

For bakre plassering i bokser brukes en egen fold-over-logikk i [`_select_rear_fold_solution()`](../src/ik/solver.py:226). Dette gjør at armen kan nå bakover uten å rotere basen ukontrollert rundt. Designvalget støtter trygg sortering til bokser bak roboten.

### Forward kinematics

Metoden [`forward_kinematics()`](../src/ik/solver.py:827) gjør det motsatte av IK: den tar motorsteg og beregner hvor kloen havner i `x`, `y` og `z`. Dette er viktig av tre grunner:

1. Det gjør det mulig å verifisere om IK-løsningen faktisk peker på riktig sted, fordi motorstegene må gi samme posisjon tilbake.
2. Det brukes ved limp-mode eller manuell kalibrering, der en fysisk posisjon kan leses tilbake og spilles av igjen, fordi målte posisjoner da kan gjentas.
3. Det brukes i tester for å oppdage feil i vinkelkonvensjoner, fordi små fortegnsfeil kan gi store bevegelser.

Forward kinematics er derfor en sentral del av kvalitetssikringen. Hvis IK og FK ikke stemmer overens, betyr det at armens matematiske modell ikke representerer den fysiske armen godt nok.

### Viktige designvalg i IK-koden

De viktigste designvalgene er:

- **Geometrisk IK i stedet for maskinlæring eller numerisk optimalisering.** Dette gir forutsigbare og raske beregninger, fordi armgeometrien er kjent og lett å forklare, dokumentert i [docs/decisions/002-4dof-geometry.md](decisions/002-4dof-geometry.md).
- **Dynamisk wrist-pitch.** Dette lar armen vippe håndleddet fremover for å nå mål som ellers ville vært utenfor rekkevidde, fordi håndleddet kan bidra med ekstra rekkevidde.
- **Sag-kompensasjon.** Koden løfter `z`-målet, fordi armen synker mer jo lenger den strekker seg.
- **Fail-closed strict mode.** Kritiske ruter stopper med tydelig feil, fordi en avvist rute er tryggere enn en farlig motorposisjon.
- **Joint limits.** Motorene får aldri bevisst kommandoer utenfor definerte trygge områder, fordi grensene beskytter både motorer og mekanikk.

---

## Sag-kalibrering: [src/calibration/03_sag.py](../src/calibration/03_sag.py:1)

[src/calibration/03_sag.py](../src/calibration/03_sag.py:1) brukes for å måle hvor mye robotarmen synker på grunn av tyngdekraft når den strekker seg ut. Målingen sammenligner teoretisk høyde fra IK med faktisk høyde på den fysiske armen.

### Hvordan scriptet fungerer

Scriptet beveger armen til flere horisontale rekkevidder definert i [`TEST_REACHES`](../src/calibration/03_sag.py:120). For hver rekkevidde måler brukeren den faktiske høyden på kloen. Målingene samles inn av [`collect_data()`](../src/calibration/03_sag.py:162), og hver posisjon måles flere ganger med [`read_averaged_measurement()`](../src/calibration/03_sag.py:144) for å redusere målefeil.

Deretter tilpasses både en lineær og en kvadratisk modell i [`fit_models()`](../src/calibration/03_sag.py:195). Resultatet lagres av [`save_calibration()`](../src/calibration/03_sag.py:311) til [src/ik/sag_calibration.json](../src/ik/sag_calibration.json). Denne filen lastes automatisk i [`_load_sag_calibration()`](../src/ik/solver.py:142).

### Betydning for systemet

Sag-kalibreringen gjør at roboten kan gripe baller mer presist. Korrigeringen reduserer avviket mellom beregnet høyde og faktisk høyde, spesielt ved lang rekkevidde.

---

## SCAN_POSE-kalibrering: [src/calibration/02c_scan_pose.py](../src/calibration/02c_scan_pose.py:1)

[src/calibration/02c_scan_pose.py](../src/calibration/02c_scan_pose.py:1) er et interaktivt verktøy for å finne en god scan-posisjon for armen. Kameraet er montert på håndleddet, og verktøyet brukes for å se arbeidsområdet fra samme posisjon hver gang.

### Hvordan verktøyet brukes

Ved oppstart flyttes armen til gjeldende `SCAN_POSE` fra konfigurasjonen. Kameraet åpnes, og brukeren kan justere motorene med tastaturet. Funksjonen [`draw_overlay()`](../src/calibration/02c_scan_pose.py:138) legger motorverdier og kontrolltaster oppå kamerabildet. Når brukeren trykker Enter, lagrer [`save_scan_pose()`](../src/calibration/02c_scan_pose.py:172) de nye motorverdiene tilbake i armkonfigurasjonen.

### Hvorfor fast scan-posisjon

Fast scan-posisjon er dokumentert i [docs/decisions/003-fixed-scan-pose.md](decisions/003-fixed-scan-pose.md). Kameraet ser da arbeidsrommet fra samme vinkel. Det gir mer stabile homografi-, deteksjons- og testresultater.

---

## Touch-kalibrering: [src/calibration/09_touch_calibration.py](../src/calibration/09_touch_calibration.py:1)

[src/calibration/09_touch_calibration.py](../src/calibration/09_touch_calibration.py:1) er et omfattende kalibreringsverktøy som kobler kamerakoordinater til armens koordinatsystem. I stedet for å måle punkter med linjal, lar verktøyet robotarmen fysisk berøre kalibreringspunkter.

### Hovedidé

Kameraet ser baller i pikselkoordinater. IK-systemet trenger posisjoner i centimeter. Touch-kalibreringen bygger en homografi som oversetter mellom disse koordinatsystemene.

Arbeidsflyten er:

1. Armen flyttes til scan-posisjon med [`_move_to_scan_pose()`](../src/calibration/09_touch_calibration.py:313), fordi kameraet må se arbeidsområdet fra fast vinkel.
2. Kameraet finner baller automatisk med [`_auto_detect_balls()`](../src/calibration/09_touch_calibration.py:324), fordi punktene først må kobles til pikselkoordinater.
3. Deteksjonene gjennomsnittberegnes over mange frames for å redusere støy, fordi enkeltbilder kan variere.
4. Brukeren finjusterer kloen fysisk over hvert punkt, fordi robotens faktiske posisjon er fasiten.
5. Scriptet beregner homografi og lagrer resultatet til [src/calibration/homography_calibration.json](../src/calibration/homography_calibration.json), fordi kalibreringen må kunne brukes automatisk senere.

### Sikkerhet i touch-kalibrering

Touch-kalibreringen inneholder sikkerhet rundt overgangen fra limp-mode til IK-styrt finjustering. Funksjonen [`_limp_fine_tune_start_height()`](../src/calibration/09_touch_calibration.py:103) velger en trygg start-høyde basert på rekkevidde, mens [`_validate_ik_fk_xy()`](../src/calibration/09_touch_calibration.py:275) sjekker at den løste IK-posisjonen faktisk havner nær ønsket `x` og `y`. Hvis FK viser for stort avvik, stopper verktøyet før armen beveger seg til feil sted.

Designvalget bak denne metoden er dokumentert i [docs/decisions/004-touch-calibration-replaces-homography.md](decisions/004-touch-calibration-replaces-homography.md). Hovedpoenget er at fysisk berøring gir bedre samsvar mellom kamera og robotarm enn manuell linjalmåling.

---

## Klo- og gripetest: [src/calibration/02b_claw_grip_test.py](../src/calibration/02b_claw_grip_test.py:1)

[src/calibration/02b_claw_grip_test.py](../src/calibration/02b_claw_grip_test.py:1) tester om kloen faktisk klarer å gripe en ball, og om sensorverdiene kan brukes til å oppdage grip. Scriptet leser posisjon, last og strøm fra motorene, og bruker samme terskler som produksjonskoden.

Viktige deler er:

- [`read_claw_feedback()`](../src/calibration/02b_claw_grip_test.py:168), som henter posisjon, last og strøm for klo-motoren, fordi grip må vurderes fra motorens faktiske respons.
- [`feedback_confirms_grip()`](../src/calibration/02b_claw_grip_test.py:218), som vurderer om sensorene viser at en ball er grepet, fordi systemet trenger en enkel ja/nei-vurdering.
- [`run_grip_test()`](../src/calibration/02b_claw_grip_test.py:271), som kjører en full adaptiv lukking med logging, fordi hele gripsekvensen må prøves slik den skjer i praksis.

Hvordan adaptivt grep fungerer:

Kloen lukkes ikke rett til én fast verdi. Den lukkes gradvis i små steg, fordi ballene kan ha litt ulik størrelse og ligge litt forskjellig i kloen. Etter hvert steg leser systemet tilbakemelding fra motoren, for eksempel last, strøm, posisjon og om motoren stopper opp. Når motstanden øker, tolkes det som at kloen har truffet ballen og at objektet er grepet.

Derfor stopper ikke systemet bare blindt på en forhåndsbestemt lukkeverdi. Hvis grepet trenger litt mer sikkerhet, legges det på en liten ekstra gripemargin. Dette gjør grepet mer skånsomt enn å lukke like hardt hver gang, samtidig som det reduserer risikoen for at ballen glipper.

Dette gir en praktisk verifisering av at mekanikken, motorstrømmen og programlogikken stemmer sammen. Det er spesielt viktig fordi sorteringssystemet må vite om en ball faktisk ble plukket opp, eller om roboten skal scanne på nytt.

---

## Simulering og visualisering

### 3D-visualisering: [src/simulation/visualizer.py](../src/simulation/visualizer.py:1)

[src/simulation/visualizer.py](../src/simulation/visualizer.py:1) viser robotarmen i et 3D-vindu med `matplotlib`. Den bruker forward kinematics for å tegne base, skulder, albue, håndledd og klo. Funksjonen [`forward_kinematics()`](../src/simulation/visualizer.py:104) konverterer motorsteg til leddposisjoner, mens klassen [`ArmVisualizer`](../src/simulation/visualizer.py:309) står for rendering.

Visualiseringen kan brukes til å se om IK-løsninger ser fysisk realistiske ut før de sendes til roboten. Den viser også bokser og ruter for å synliggjøre kollisjonsfare eller urimelige bevegelser.

### Mock-seriellport: [src/simulation/mock_serial.py](../src/simulation/mock_serial.py:1)

[src/simulation/mock_serial.py](../src/simulation/mock_serial.py:1) definerer [`MockSerial`](../src/simulation/mock_serial.py:41), en falsk erstatning for `serial.Serial`. Den gjør det mulig å teste systemet uten OpenRB og motorer. Metoden [`write()`](../src/simulation/mock_serial.py:86) tar imot JSON-kommandoer på samme måte som firmware-broen, og [`readline()`](../src/simulation/mock_serial.py:234) returnerer typisk `OK` eller simulerte sensordata.

Når en visualizer kobles til, interpolerer [`_animate_move()`](../src/simulation/mock_serial.py:210) motorposisjonene mellom start og mål. Bevegelsen vises da som en animasjon i stedet for bare å hoppe mellom posisjoner.

### Rutedemo: [src/simulation/route_demo.py](../src/simulation/route_demo.py:1)

[src/simulation/route_demo.py](../src/simulation/route_demo.py:1) bygger og validerer simulerte sorteringsruter. Klassen [`RouteDemoWaypoint`](../src/simulation/route_demo.py:48) representerer et waypoint som allerede er validert av streng IK. Funksjonen [`build_rear_placement_demo_plan()`](../src/simulation/route_demo.py:171) lager en komplett rute fra pickup til bakre boks, mens [`execute_demo_plan()`](../src/simulation/route_demo.py:331) spiller ruten av i simulatoren via [`MockSerial`](../src/simulation/mock_serial.py:41).

Rutedemoen kjører hele flyten uten fysisk robot. Den bruker [`solve_strict()`](../src/ik/solver.py:554) slik at alle waypoints må være gyldige før animasjonen starter.

### Kollisjonssikkerhet for bokser: [src/simulation/bin_safety.py](../src/simulation/bin_safety.py:1)

[src/simulation/bin_safety.py](../src/simulation/bin_safety.py:1) modellerer boksene som kollisjonsvolumer. Klassen [`BinVolume`](../src/simulation/bin_safety.py:30) beskriver en boks med plassering, høyde og fotavtrykk. Den kan sjekke om et punkt ligger inni boksen med [`contains_point()`](../src/simulation/bin_safety.py:63), om en bevegelseslinje krysser boksen med [`segment_intersects()`](../src/simulation/bin_safety.py:79), og hvor stor avstand det er til boksen med [`segment_clearance_cm()`](../src/simulation/bin_safety.py:114).

Funksjonene [`find_claw_bin_point_clearance_violation()`](../src/simulation/bin_safety.py:238) og [`find_claw_bin_segment_clearance_violation()`](../src/simulation/bin_safety.py:259) brukes til å avvise ruter der kloen er for nær en boks. Kontrollen gir en enkel sikkerhetsmargin før fysisk kjøring.

---

## Diagnostikk av motorer og hardware

### Motoroppdagelse: [src/diagnostics/diagnose_motors.py](../src/diagnostics/diagnose_motors.py:1)

[src/diagnostics/diagnose_motors.py](../src/diagnostics/diagnose_motors.py:1) kontrollerer om alle Dynamixel-motorene svarer. Scriptet sender en `diagnose`-kommando til OpenRB, som pinger motorene på flere baud-rater. Funksjonen [`send_diagnose()`](../src/diagnostics/diagnose_motors.py:105) sender kommandoen, og [`print_report()`](../src/diagnostics/diagnose_motors.py:134) skriver ut en tydelig rapport.

Rapporten viser:

- hvilke motor-IDer som ble funnet
- hvilken motormodell som svarer
- hvilken baud-rate motoren bruker
- om motoren mangler eller har feil konfigurasjon

Rapporten gir et kort feilsøkingsgrunnlag når armen ikke beveger seg som forventet.

### Hardware error flags: [src/diagnostics/check_motor_errors.py](../src/diagnostics/check_motor_errors.py:1)

[src/diagnostics/check_motor_errors.py](../src/diagnostics/check_motor_errors.py:1) leser hardware-feil fra motorene. Funksjonen [`parse_error_status()`](../src/diagnostics/check_motor_errors.py:35) dekoder bitflagg for blant annet input-spenning, overoppheting, encoder-feil, elektrisk sjokk og overload.

Dynamixel-feil kan ligge låst i motoren og kreve 12V power cycle for å nullstilles. Scriptet gir en praktisk forklaring når en motor blinker rødt eller ikke reagerer.

### Live-debugging: [src/diagnostics/stream_debug.py](../src/diagnostics/stream_debug.py)

[src/diagnostics/stream_debug.py](../src/diagnostics/stream_debug.py) brukes til live debugging. Den viser kamerabilde, masker, deteksjonsstatus og relevant sanntidsinformasjon. Den kan brukes til å se samspillet mellom fysisk arm, kamera, scan-posisjon og deteksjon.

---

## Tester og verifisering

### IK-tester: [tests/test_ik_solver.py](../tests/test_ik_solver.py:1)

[tests/test_ik_solver.py](../tests/test_ik_solver.py:1) er hovedtesten for IK-koden. Den dekker blant annet:

- rekkevidde og unreachable targets i [`TestReachability`](../tests/test_ik_solver.py:74)
- leddgrenser i [`TestJointLimits`](../tests/test_ik_solver.py:118)
- streng IK i [`TestStrictSolve`](../tests/test_ik_solver.py:149)
- symmetri mellom `+y` og `-y` i [`TestSymmetry`](../tests/test_ik_solver.py:255)
- geometrisk konsistens og FK round-trip i [`TestGeometricConsistency`](../tests/test_ik_solver.py:298)
- delvis bevegelse i [`TestPartialMove`](../tests/test_ik_solver.py:384)

En hjelpefunksjon er [`_forward_kinematics_xy()`](../tests/test_ik_solver.py:20), som reverserer deler av IK-matematikken for å sjekke om motorstegene gir en plausibel posisjon. Testene dokumenterer hvilke egenskaper IK-løseren må bevare.

### Simuleringstester

[tests/test_simulation_route_demo.py](../tests/test_simulation_route_demo.py) tester at rutedemoen tolker kalibreringsdata riktig og bygger trygge ruter. [tests/test_touch_calibration_safety.py](../tests/test_touch_calibration_safety.py) tester sikkerhetslogikk rundt touch-kalibrering. [tests/test_adaptive_grip.py](../tests/test_adaptive_grip.py) tester grep-logikken og at sensorfeedback vurderes riktig.

Disse testene verifiserer at systemet feiler trygt. De dekker situasjoner der posisjoner, sensorer eller kalibrering er feil.

---

## Hvordan U-koden henger sammen med resten av systemet

U-koden fungerer som bindeledd mellom fysisk robot og høyere systemlogikk:

1. Vision-systemet finner en ball og gir et mål i centimeter.
2. [`ArmIK`](../src/ik/solver.py:37) beregner motorsteg for målet.
3. Hovedløkken sender motorstegene til OpenRB-firmwaren.
4. Kalibreringsfilene, for eksempel [src/ik/sag_calibration.json](../src/ik/sag_calibration.json), gjør at beregningene passer bedre med den faktiske armen.
5. Simulering og tester brukes til å verifisere ruter før fysisk kjøring.
6. Diagnostikkverktøy brukes når motorer, strøm, temperatur eller posisjon ikke oppfører seg riktig.

Kjernen i U sin muntlige forklaring er hvordan fysisk mål, IK-beregning, kalibrering og motorstyring henger sammen. Støtteverktøyene kan nevnes kort for å vise hvordan løsningen kontrolleres.

---

## Oppsummering

For muntlig bachelorforsvar bør hovedvekten ligge på geometrisk IK, FK-verifisering, sag-kompensasjon som del av solver-modellen, touch-kalibrering og adaptivt grep. Simulering, rutevalidering, tester og motor-diagnostikk kan omtales kort som støtte rundt systemet.

I bachelorprosjektet kan denne delen beskrives som arbeidet som gjør robotens bevegelser presise, forklarbare og trygge.
