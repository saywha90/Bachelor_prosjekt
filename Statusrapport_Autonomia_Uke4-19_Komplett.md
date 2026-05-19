# Samlet Statusrapport - Prosjekt Autonomia
**Periode:** Uke 4 (20. januar) – Uke 19 (10. mai 2026)
**Gruppeleder:** Umran

Dette er den samlede statusrapporten for prosjektet Autonomia, som dekker hele prosjektperioden fra oppstart i uke 4 og frem til dags dato. Rapporten inneholder de individuelle oppdateringene fra teamets medlemmer i henhold til lærerens krav.

*(Merk: KI/AI har blitt benyttet til å strukturere og sette sammen denne rapporten for å sikre et enhetlig format og god lesbarhet. Alt faglig innhold og all tekst reflekterer imidlertid kun medlemmenes eget arbeid og egne vurderinger.)*

---

## Periodeinndeling
| Periode | Datoer | Beskrivelse |
|---------|--------|-------------|
| Uke 4–5 | 20. januar – 1. februar | Oppstart og konseptarbeid, inkl. 1. presentasjon 5. februar |
| Uke 6–11 | 2. februar – 15. mars | Hovedfase 1 – Mekanisk design, elektrisk arkitektur og bildedeteksjon |
| Uke 12–13 | 16. mars – 29. mars | Motorarbeid og 2. presentasjon (26. mars) |
| Uke 14–17 | 30. mars – 26. april | Hovedfase 2 – Hardware-integrasjon og testing |
| Uke 18 | 27. april – 3. mai | Dokumentasjon og videreutvikling |
| Uke 19 | 4. mai – 10. mai | Sluttspurt og siste tester |
| Uke 22 | 27. mai | 3. presentasjon (sluttpresentasjon) |

---

## 1. Individuell Rapport: Umran
**Rolle:** Gruppeleder, Logistikkansvarlig og Utvikler (Hardware/Software)

### Uke 4–5 (20. januar – 1. februar): Oppstart
I oppstartsfasen deltok jeg på introduksjonsmøter og planleggingsmøter, samt veiledningsmøte med Dag. Jeg jobbet med å sette opp prosjektets nettside og forberede første presentasjon (5. februar, PowerPoint), samtidig som jeg satte meg inn i prosjektets tekniske krav og rammer.

### Uke 6–11 (5. februar – 15. mars): Hovedfase 1 – Mekanisk, elektrisk og programvare
Arbeidet mitt fulgte en kronologisk rød tråd fra mekanisk design, til avklaringer med oppdragsgiver, og videre inn i en dyp teknisk analyse av systemets elektronikk.

#### Arbeid med CAD og mekanisk design
Det første vi tok tak i etter forrige presentasjon (1. presentasjon, 5. februar) var det mekaniske. Jeg bistod Mohammed med å komme i gang med CAD-modelleringen for å sikre rask fremdrift. Vi fant sammen en eksisterende robotarm for inspirasjon, kom frem til en ferdig løsning, modellerte den og printet den ut. Jeg utarbeidet også en første handleliste basert på denne nye armen.

#### Veiledermøte og ny retning
Deretter hadde vi et møte med oppdragsgiver. Der fikk vi beskjed om at de foretrakk at vi heller tok utgangspunkt i fjorårets ferdigstilte robotarm. Vi måtte derfor legge den nyprintede versjonen vår til side.

#### Dybdeanalyse av forrige gruppes rapport
Med beskjed om å bruke fjorårets arm, undersøkte og analyserte jeg fjorårets rapport i detalj for å finne ut nøyaktig hvilke svakheter vi måtte utbedre. Hovedfunnene var:
1.  **Unøyaktig invers kinematikk:** Armen slet med posisjonsavvik på grunn av numeriske unøyaktigheter og mekaniske toleranser, som gjorde at den ofte bommet på målet (> 5 graders feil).
2.  **Software/Hardware inkompatibilitet:** De oppgraderte fra Raspberry Pi 4 til 5, noe som skapte store problemer da operativsystemet til Pi 5 manglet støtte for ROS 2 Humble. Dette førte til forsinkelser og at meldingskjeden sviktet i testing.
3.  **Mekaniske svakheter:** De 3D-printede festene deformerte seg under belastning (testet med 190 g payload) på grunn av den lange rekkevidden.
4.  **Strømforsyning (Power Supply Limitations):** Arduino Megaen klarte ikke levere nok strøm til den analoge klo-servoen, og deres 3A lab-strømforsyning ble en stor flaskehals som krevde ustabile nødløsninger.

*Min løsning på de mekaniske svakhetene:* For å forbedre fjorårets design har jeg foreslått å korte ned armen fra 586 mm til 520–540 mm. Dette vil redusere vektstangprinsippets påkjenning på leddene og motorene betydelig, samtidig som vi opprettholder rekkevidden på 512 mm som kreves for fremtidig montering på omni-bilen "Baldr".

#### Elektrisk design og strømanalyse
Med utgangspunkt i analysen satte jeg meg dypt inn i den elektriske delen og tok hovedansvar for å forbedre den elektriske arkitekturen:

**1. Ny klo-servo og strømforsyning (adapter):**
*   *Problem identifisert:* Forrige gruppe brukte en liten 5V analog servo til kloen styrt via Arduino Mega og en PCA9685-driver. Megaen klarte ikke å levere nok strøm, noe som krevde en ustabil nødløsning.
*   *Min løsning:* Jeg besluttet å bytte ut den analoge 5V-servoen i kloen med en Dynamixel-servo (XM430-W210). Valget falt på XM430 fremfor den lettere XL430, fordi kloa trenger høyere dreiemoment for å gripe ballene sikkert uten at motoren staller.
*   *Beregning av strømforsyning:* For å drive de totalt 5 Dynamixel-servoene har jeg beregnet det teoretiske maksimale strømtrekket (Stall Current) for den faktiske motorfordelingen:
    *   Motor 1 – Base (1x XM430-W210): 2,3 A
    *   Motor 2 – Skulder (1x XM540-W150): 4,4 A
    *   Motor 3 – Albue (1x XM430-W210): 2,3 A
    *   Motor 4 – Håndledd (1x XL430-W250): 1,3 A
    *   Motor 5 – Klo (1x XM430-W210): 2,3 A
    *   **Totalt:** ~12,6 Ampere.
*   *Valg av adapter:* Jeg kom frem til at et **12V 10A (120W) adapter** er den optimale løsningen. Normal drift trekker ca. 2–5A. Dersom alle motorene staller (12,6 A), trigger 10A-strømforsyningen overstrømsvernet, som fungerer som en innebygd sikring.

**2. Stjernetopologi (løsning på spenningsfall og daisy-chain):**
*   *Problem identifisert:* Forrige gruppe kjørte strøm og data gjennom én lang Daisy Chain. Dette skapte en farlig trakteffekt der hele den teoretiske stallstrømmen for vår faktiske motorpakke (~12,6 A) kunne blitt presset gjennom én tynn JST-kontakt (ratet for 3–5A), med risiko for overoppheting, spenningsfall (brownout) og ustabil motordrift.
*   *Min løsning – Hybrid stjernetopologi:* Jeg designet en hybrid stjernekobling fra kontrollkortet (opprinnelig Dynamixel Shield, senere realisert på OpenRB-150) som fordeler strømmen over to separate grener:
    *   **Gren 1:** Fra kontrollkortet til Motor 1 – Base (XM430-W210, 2,3 A) og Motor 2 – Skulder (XM540-W150, 4,4 A), koblet sammen. Samlet maks strømtrekk for denne grenen er 6,7 A. Disse to motorene sitter fysisk ved bunnen av armen og deler dermed én kort kabelvei fra strømkilden.
    *   **Gren 2:** Fra kontrollkortet til Motor 3 – Albue (XM430-W210, 2,3 A), deretter videre i daisy-chain til Motor 4 – Håndledd (XL430-W250, 1,3 A) og Motor 5 – Klo (XM430-W210, 2,3 A). Samlet maks strømtrekk for denne grenen er 5,9 A. Disse motorene sitter lenger oppe på armen og trekker generelt mindre strøm i normal drift.
*   *Resultat:* Strømmen er fordelt over to grener i stedet for én lang kjede, slik at ingen enkelt kabel bærer hele systemets 12,6 A. Datakommunikasjonen (TTL half-duplex) går fortsatt i serie via Dynamixel-protokollen, men strømfordelingen er avlastet. Denne topologien ble senere realisert på OpenRB-150, som har dedikerte TTL-porter og dermed passer godt til stjerneopplegget.

### Uke 12–13 (16. mars – 29. mars): Motorarbeid og 2. presentasjon (26. mars)
I uke 12 jobbet jeg intensivt med å sette opp og konfigurere de fem Dynamixel-servomotorene. Dette inkluderte bruk av **Dynamixel Wizard 2.0** for å verifisere firmware, tildele unike motor-ID-er (1–5), og stille inn baudrate (57 600 bps) på samtlige motorer. Under dette arbeidet oppdaget jeg at én motor (XM430) ikke responderte – feilsøkingen viste at den hadde fått korrupt EEPROM-data, og ble løst med factory reset via Wizard-verktøyet. Jeg gjennomførte også grundig research på forumer og YouTube-videoer angående Dynamixel Shield, korrekt kabling og kommunikasjonsoppsett, og dokumenterte funnene. Jeg skrev også rapport for det jeg hadde gjort frem til da. I uke 13 fortsatte debugging-arbeidet med fokus på å verifisere at alle motorer svarte korrekt på posisjonskommandoer. Parallelt designet og ferdigstilte jeg PowerPoint-presentasjonen, manus og øving, og gjennomførte andre presentasjon (26. mars).

### Uke 15 (7. april – 12. april): Lodding og research på OpenRB-150
I uke 15 startet jeg med å sette meg inn i lodding ved å se gjennom flere instruksjonsvideoer. Jeg fikk loddeutstyret tirsdag, og torsdag begynte jeg å lodde OpenCM 9.04-kortet vi fikk av Steven, med tanke på å bruke det som styringsenhet for Dynamixel-motorene. Jeg dokumenterte loddearbeidet med video. Dessverre fungerte ikke OpenCM 9.04-løsningen i praksis, og jeg brukte derfor fredag og lørdag på å undersøke andre alternativer og sammenligne med utstyret vi hadde tilgjengelig. Søndag bestilte jeg utstyret selv fra en tysk nettside – OpenRB-150, en Dynamixel Shield (som backup) samt en ny XL430-motor.

### Uke 16 (13. april – 17. april): Mottatt utstyr og første programvarearbeid
Mandag og tirsdag i uke 16 jobbet jeg med kode i påvente av at varene skulle komme. Jeg laget en enkel sketch av invers kinematikk, og kodet broen mellom kameraet og maskinen min. Onsdag 15. april ankom utstyret. Samme dag sentrerte jeg alle motorene, ga dem ID 1–5 og fikset de gamle motorene som ikke fungerte (factory reset). Jeg gjennomførte også møte med Merete denne uken. Torsdag testet jeg koden jeg hadde laget for broen mellom maskinen og kameraet.

### Uke 17 (20. april – 26. april): Sammenstilling og første robottesting
Mandag 20. april satte jeg sammen robotarmen med Mohammed. Vi hadde også møte med Joakim. Allerede ved første testing var armen for nær pulten – et tilbakevendende problem som har gjentatt seg gjennom prosjektet. Tirsdag fortsatte feilsøking og testing, og armen kom etter hvert ned til ballen. Onsdag jobbet jeg med kodebasen og deltok i veiledermøte (delvis sykdom denne dagen). Lørdag gjennomførte vi videre testing av roboten, men den ville fortsatt ikke komme nær nok ballene.

#### Oppsummering av hardware-bytte (uke 15–17)
OpenRB-150 fungerte svært godt fra første test. På bakgrunn av dette besluttet vi i gruppen å erstatte den tidligere Arduino + Dynamixel Shield-kombinasjonen med OpenRB-150, for å forenkle systemet både elektrisk og programvaremessig. Som testansvarlig gjennomførte jeg full testing av armen så snart den var klar. Resultatene viste at motorene fungerte fint, men to nye problemer kom frem:
1.  **Kloa knekker:** Selve kloa har knekt flere ganger under bruk, og må enten redesignes eller forsterkes mekanisk.
2.  **Presisjonsavvik:** Armen er litt upresis ved posisjonering over ballene.

### Uke 18 (27. april – 3. mai): Intensiv testing og dokumentasjon
Uken inneholdt mye effektiv arbeidstid (ca. 47 timer) med testing, kodeforbedringer og dokumentasjon. Jeg deltok også på møte med Dag (veileder).

#### Kodebidrag — IK, kalibrering, visualisering og diagnostikk
Det meste av den tekniske koden ble ferdigstilt og raffinert i denne perioden:

*   **Invers kinematikk (`src/ik/solver.py`):** Ferdigstilte den geometriske IK-solveren for 4-DOF armen. Implementerte dynamisk wrist-pitch loop for utvidet rekkevidde, sag-kompensasjon (linær + kvadratisk modell auto-lastet fra `sag_calibration.json`), joint limits og reach clamping for å forhindre maskinvarefeil.
*   **3D-visualisering (`src/simulation/visualizer.py`):** Utviklet live 3D matplotlib-rendering av armen via forward kinematics, med Poly3DCollection og ghost trail. Fungerer som både demo og sanity-check mot IK-solveren.
*   **Touch-kalibrering (`src/calibration/09_touch_calibration.py`):** Implementerte interaktiv touch-kalibrering av homografi-matrisen med auto-deteksjon av ballsentre, frame-averaging, N-punkt RANSAC og reprojeksjonsfeil-rapport. Erstatter den eldre linjals-baserte metoden.
*   **Sag-kalibrering (`src/calibration/03_sag.py`):** Skrev script som måler gravitasjons-droop på flere rekkevidde-distanser, fitter linær + kvadratisk modell, og lagrer `sag_calibration.json` (auto-lastet av `ArmIK`).
*   **SCAN_POSE-kalibrering (`src/calibration/02c_scan_pose.py`):** Implementerte manuell kalibrering av SCAN_POSE med WASD-styring — definerer hvor armen parkerer kameraet for å se hele arbeidsrommet.
*   **IK-testsuite (`tests/test_ik_solver.py` + `tests/conftest.py`):** Skrev omfattende unit-tester med FK round-trip-verifisering, symmetri-bevis (mirror-Y), sweep-validering over alle bin-posisjoner, dynamic pitch-tester og edge cases. Opprettet pytest-fixtures i `conftest.py`.
*   **Virtuelt IK-testrammeverk (`scripts/manual_tests/ik_virtual_demo.py`):** Utviklet virtuelt test-rammeverk som mater inn fiktive kamera-koordinater, printer JSON-output og flagger mistenkelige hopp — ren matematikk-validering uten hardware.
*   **M3 termal-beskyttelse (`tests/test_main_m3_thermal.py`):** Skrev tester for termal-beskyttelseslogikken — strømlesing, SCAN_POSE current-limit og torque-relax for XM430-W210 i albueledd som blir varm i SCAN_POSE (0,47 A kontinuerlig).
*   **Motor-diagnostikk:** Utviklet tre diagnostiske verktøy:
    *   `src/diagnostics/diagnose_motors.py` — pinger alle 5 Dynamixel-motorer ved flere baud-rater (57600, 115200, 1M).
    *   `src/diagnostics/check_motor_errors.py` — leser hardware-error-flags (overheat, overload, voltage, encoder, electrical shock).
    *   `src/diagnostics/stream_debug.py` — live-stream av motor-data (posisjon, last, temperatur, strøm) for sanntids-feilsøking.

#### Dokumentasjon
Skrev flere designvalg-dokumenter (ADR) og teknisk dokumentasjon:
*   `docs/decisions/002-4dof-geometry.md` — designvalg for hvorfor geometrisk IK fremfor numeriske/ML-tilnærminger, med sammenligningstabell.
*   `docs/decisions/003-fixed-scan-pose.md` — designvalg for hvorfor fast SCAN_POSE fremfor adaptiv scanning.
*   `docs/decisions/004-touch-calibration-replaces-homography.md` — hvorfor touch-kalibrering erstatter linjals-måling.
*   `docs/troubleshooting.md` — feilsøkingsguide for IK-relaterte problemer, SCAN_POSE-justering og M3 termal-issues.
*   `docs/hardware.md` — maskinvarespesifikasjoner og hardware-valg (Dynamixel XM430-W210/XM540-W150/XL430-W250), link-lengder og kabling.

### Uke 19 (4. mai – 10. mai): Sluttspurt
Mandag hadde jeg møte med veileder Joakim. Etter møtet fortsatte jeg testingen av robotarmen. Dessverre traff armen pulten igjen under testing og knakk – noe som har skjedd flere ganger gjennom prosjektet. Tirsdag printet vi ut en ny arm og satte den sammen, og onsdag fortsatte vi testingen. Under testingen ble også XL430-motoren ødelagt fordi den satte seg fast i bin-kurven. Derfor bestilte jeg en ny motor, som ankom i dag, 15. mai. Før erstatningen kom, brukte vi den ødelagte motoren til å finne og rette den underliggende årsaken til at den satte seg fast, slik at samme feil ikke skal oppstå igjen. I tillegg utviklet jeg og Farden en løsning der armen ikke lenger trenger å snu seg fysisk for å sortere ballene – den kan i stedet invertere seg helt bakover, noe som forenkler bevegelsesmønsteret.

### Hva som er planlagt fremover
*   **Løsning på kloproblemet:** Sammen med Mohammed se på en mer robust løsning for kloa.
*   **Videre presisjonsforbedring:** Fortsette finjusteringen av kalibrering og kinematikk.
*   **Kinematikk:** Ferdigstille koden for invers og forover-kinematikk.
*   **Samkjøring med bildedeteksjon:** Fortsette samarbeidet med Ole.
*   **Gruppeledelse:** Følge opp gruppens fremdrift mot sluttpresentasjonen (27. mai).

---

## 2. Individuell Rapport: Mohammed
**Rolle:** Mekanisk Design og Produksjon

### Uke 4–5 (20. januar – 1. februar): Oppstart og konseptarbeid
Deltok på introduksjonsmøter, konsepttegning og veiledningsmøte med Dag. Jeg jobbet med konseptet i Blender, dimensjonerte bilen fra gruppen for å vite sluttlengden til armen, og begynte med å dimensjonere armen i XYZ-retning.

### Uke 6–11 (5. februar – 15. mars): Hovedfase 1 – CAD og første prototyper
*   **CAD-design i SolidWorks:** Jeg 3D-designet roboten i SolidWorks og utviklet tre ulike konsepter som ble presentert for og vurdert av gruppen.
*   **Forberedelse av klo-mekanisme:** Gjennomførte undersøkelser av robotens klo-arm, gjorde nødvendige beregninger og tegnet flere eksemplarer som grunnlag for 3D-modellering.

### Uke 12–13 (16. mars – 29. mars): Konseptferdigstilling og 2. presentasjon (26. mars)
Jeg ferdigstilte konsept 1 og konsept 2, og brukte resten av perioden på rapportskriving samt arbeid med PowerPoint og manus til 2. presentasjon (26. mars).

### Uke 14–17 (30. mars – 26. april): Motormodellering og redesign

#### Uke 14–15 – Motormodellering og innledende beregninger
Jobbet i SolidWorks med å lage CAD-modeller av motorene XM430 og XL430. I tillegg startet jeg arbeidet med å designe braketter til armen. For å sikre at designet er strukturelt forsvarlig, gjennomførte jeg torque-beregninger for hånd, samt enkle FEM-analyser basert på den gamle robotarmen.

#### Uke 16–17 – Tilpasning til nye lengdekrav og redesign
Jobbet med å møte de nye lengdekravene til roboten. Dette omfattet brakettene, platen som roboten står på, leddene, samt kameraholderen og plasseringen av denne. Jeg redesignet de fleste delene av robotarmen, med spesielt fokus på leddene, brakettene og huset til armen.

### Uke 18 (27. april – 3. mai): Videre forbedring og ferdigstilling
Jobbet videre med å forbedre robotarmen ut fra problemene som dukket opp underveis (dimensjonsfeil og andre justeringer). Planen var å bli ferdig med resten av armen, slik at dataingeniørene kunne fortsette sine tester. Samtidig skulle jeg ferdigstille plattformen som roboten står på.

### Uke 19 (4. mai – 10. mai): Reservedelsproduksjon og topologi-research
Printet ut nye deler til robotarmen, blant annet etter at den eksisterende armen knakk under testing tidligere i uken. I tillegg har jeg begynt å undersøke hvordan vi kan produsere armen i SLS (Selective Laser Sintering) ved bruk av topologi-optimalisering, slik at vi kan oppnå en sterkere og mer optimalisert konstruksjon i fremtidige iterasjoner.

### Hva som er planlagt fremover
*   **Topologi-optimalisering:** Fortsette utredningen av SLS-print med topologi-optimalisering.
*   **Produksjonsstøtte:** Printe ut eventuelle nye deler etter behov.
*   **Rapportering:** Ferdigstille min del av den tekniske rapporten.

---

## 3. Individuell Rapport: Ole Aleksander Hageløkken
**Rolle:** Dokumentasjonsansvarlig og Utvikler (Bildedeteksjon)

### Uke 4–5 (20. januar – 1. februar): Prosjektstruktur og dokumentasjon
Etablerte GitHub og tidslinje, opprettet Scrum-dokumenter, kanban-board og møtereferater. Utarbeidet første utkast av fremdriftsplan for semesteret, og deltok i veiledningsmøte med Dag. Lekte litt med bildegjenkjenning som forberedelse til den tekniske rollen min.

### Uke 6–11 (5. februar – 15. mars): Hovedfase 1 – Strukturering og maskinsyn
*(Merk: Hadde 2 ukers fravær i denne perioden grunnet sykdom og jobbreise.)*

*   **Prosjektorganisering og metode (Scrum):** Deltok på Scrum-kurs og innførte metodikken i gruppen, inkludert Trello-board, daily stand-up, sprintdokumentasjon og strukturert arbeidsflyt (To Do / Doing / Done).
*   **Møteledelse og struktur:** Kalte inn til samtlige møter, utarbeidet møtereferater og etablerte faste møteserier med oppdragsgiver og veileder.
*   **Dokumentasjon og styring:** Opprettet felles OneDrive og timeregistrering. Utarbeidet kravspesifikasjon, testspesifikasjon, PRD (Product Requirements Document), sprintdokumenter, samt skrev rapport til 1. presentasjon (5. februar).
*   **Teknisk bidrag (Maskinsyn):** Påbegynt utvikling av kode for bildedeteksjon (Python) og opprettet delt GitHub-repository. Bidro til beslutningen om bruk av Raspberry Pi.

### Uke 12–13 (16. mars – 29. mars): Bildedeteksjon og 2. presentasjon (26. mars)
*(Merk: Hadde eksamen i starten av uke 12.)*

I uke 12 jobbet jeg intensivt med bildedeteksjon, testing og dokumentasjon, samt deltok i møte med veileder. I uke 13 hadde jeg lange dager med dokumentasjon og rapportarbeid, deltok i møte med oppdragsgiver, og brukte tiden mot slutten av uken på å strukturere Final Report. Lørdag testet jeg OAK Series 2-kameraet og la det til i koden, og gikk gjennom kode linje for linje.

### Uke 14–17 (30. mars – 26. april): Bildedeteksjon og prosjektstyring
*(Merk: 2 dagers frafall grunnet todagerskurs på jobb.)*

#### Teknisk arbeid – Bildedeteksjon
Kodet ferdig bildedeteksjonen og kjørte flere tester hjemme med OAK-kameraet, Raspberry Pi 5 og koden. Testingen fungerte fint, og oppsettet er stabilt. I tillegg jobbet jeg sammen med Umran om å samkjøre bildedeteksjonskoden med Arduino-koden for servostyring.

Utviklet den komplette vision-pipelinen bestående av følgende moduler:
*   `src/vision/camera.py` — OAK-D S2 kamera-wrapper med DepthAI-pipeline for frame-grabbing, oppløsning og fokal-lengde.
*   `src/vision/detector.py` — `SimpleBallDetector` med ensemble av HSV-segmentering og Hough Circle-deteksjon, adaptiv lysjustering, multi-ball tracking med track_id og persistens, samt konfidens-scoring.
*   `src/vision/classifier.py` — farge-klassifisering (rød/blå/ukjent) med shape- og color-confidence.
*   `src/ik/vision_bridge.py` — bro mellom OAK-D piksel-koordinater og IK arm-frame (cm) via homografi-transformasjon.
*   `src/config/vision.py` — vision-konstanter (kamera-oppløsning, HSV-grenser, ball-radius, konfidens-terskler).

Utviklet også kalibrerings- og treningsverktøy:
*   `src/calibration/04_hsv_tuner.py` — interaktiv HSV-tuner med trackbars for å finne fargegrenser.
*   `src/calibration/05_hsv_refine.py` — forfining av HSV-grenser med statistisk analyse.
*   `src/calibration/06_homography.py` — manuell homografi-kalibrering (eldre metode, senere erstattet av touch-kalibrering).
*   `src/calibration/07_vision_offset.py` — finjustering av kamera-til-skulder offset.
*   `src/training/capture_data.py` — innsamling av treningsdata for fargeklassifikatoren.
*   `src/training/train_classifier.py` — trening av fargeklassifikator-modellen.
*   `src/diagnostics/diagnose_detection.py` — live-debugging av deteksjonspipelinen.
*   `scripts/manual_tests/enhanced_detector_demo.py`, `oak_v3_demo.py` og `backend_check.py` — demo- og testskript for visjonsystemet.

#### Dokumentasjon og rapportarbeid
Strukturerte ferdig rapporten og videreformidlet dette til gruppen. Alle medlemmene fikk tildelt sine seksjoner å skrive i. Selv dokumenterte jeg en del i den endelige rapporten, samt la ut tilleggsdokumentasjon på OneDrive. Jeg skrev møtereferater for samtlige møter i perioden.

#### Prosjektstyring og møter (Scrum)
*   Satt opp nytt Trello-board for sprint 4.
*   Gjennomført retrospective med gruppen.
*   Deltatt i møte med veileder og oppdragsgiver.
*   Gjennomført møte med gruppen og Merete angående kommunikasjonsproblemer.

### Uke 18 (27. april – 3. mai): Dokumentasjon og daily stand
Jobbet med oppdatert dokumentasjon, daily stand med gruppen og laging av møtereferater, samt gjennomgang av kode. Deltok i møte med både oppdragsgiver og veileder.

#### Teknisk dokumentasjon
Skrev og ferdigstilte flere sentrale dokumentasjonsfiler for visjonsystemet:
*   `docs/calibration.md` — komplett kalibrerings-guide fra HSV via homografi til touch-kalibrering.
*   `docs/vision-history.md` — historikk og evolusjon av vision-pipelinen gjennom prosjektet.
*   `docs/decisions/001-hsv-over-cnn.md` — designvalgsdokument (ADR) som dokumenterer hvorfor HSV-deteksjon ble valgt fremfor CNN-basert klassifisering.

### Uke 19 (4. mai – 10. mai): Forberedelse til sluttpresentasjon (27. mai)
Jobbet med dokumentasjon og forberedelser knyttet til testresultater og rapport.

### Hva som er planlagt fremover
*   **Ferdigstilling av testdokumentasjon:** Fokusere på å ferdigstille dokumentasjonen av testingen.
*   **Oppdatering av rapporten:** Oppdatere rapporten basert på testresultater.
*   **Forberedelse til sluttpresentasjon (27. mai):** Sørge for at mine deler er klare til gjennomgang og sluttpresentasjon (27. mai).
*   **Prosjektstyring:** Fortsette å drifte Scrum-prosessen.

---

## 4. Individuell Rapport: Farden
**Rolle:** Utvikler (Motorstyring/Kommunikasjon) og Systemintegrasjon

### Uke 4–5 (20. januar – 1. februar): Oppstart og innledende research
Deltok på introduksjonsmøter og veiledning. Jobbet med undersøkelse av prosjektet, risikoplan og innledende research på Raspberry Pi.

### Uke 6–11 (5. februar – 15. mars): Hovedfase 1 – Kommunikasjon og systemarkitektur
*   **Kommunikasjon mellom systemer:** Undersøkte hvordan kommunikasjonen mellom Arduino og Raspberry Pi kunne implementeres ved bruk av seriell kommunikasjon.
*   **Styring av Dynamixel-motorer:** I tett samarbeid med Umran fokuserte jeg på programmering og styring av Dynamixel-motorene, inkludert konfigurasjon, testing av kode for posisjons- og bevegelseskommandoer, samt feilsøking.
*   **Bildebehandling (teoretisk grunnlag):** Siden Raspberry Pi ennå ikke var tilgjengelig, jobbet jeg teoretisk med bildebehandling. Undersøkte hvordan Python og OpenCV kunne benyttes for fargebasert bildeanalyse.
*   **Dokumentasjon og arkitektur:** Bidro til prosjektdokumentasjonen ved å skrive og strukturere deler av rapporten, samt utforme diagrammer som visualiserer dataflyten.

### Uke 12–13 (16. mars – 29. mars): Motortesting og 2. presentasjon (26. mars)
I uke 12 hadde jeg lange dager med testing og undersøkelse av Dynamixel-motorene, inkludert intensiv feilsøking på en motor som ikke ville bevege seg. Skrev også på min del av rapporten. I uke 13 fokuserte jeg på forberedelser til 2. presentasjon (26. mars).

### Uke 14–17 (30. mars – 26. april): Rapport, systemarkitektur og firmware

#### Rapport og dokumentasjon
Jobbet mye med hovedrapporten – spesielt systemarkitektur-delen, der jeg oppdaterte og utvidet den lagdelte arkitekturoversikten, beskrev kommunikasjonsprotokollen mellom Raspberry Pi og OpenRB-150, og dokumenterte integrasjonen mellom systemkomponentene. Bidro også til maskinlæringsdelen av rapporten.

Skrev og oppdaterte flere dokumentasjonsfiler:
*   `docs/architecture.md` – beskrivelse av systemarkitekturen Pi → OpenRB → motorer, samt Pi → kamera.
*   `docs/pi-setup.md` – oppsett av Raspberry Pi inkludert OS, drivere og avhengigheter.
*   `docs/performance.md` – ytelsesmålinger for latens, FPS og syklustid.

#### Firmware og systemintegrasjon
Jobbet med `openrb_bridge.ino`, som fungerer som USB-broen mellom Raspberry Pi og de fem daisy-chainede Dynamixel-motorene. Designet og implementerte JSON-protokollen over seriell, der Raspberry Pi sender motorposisjoner og OpenRB-150 svarer med OK.

Jobbet også med hovedløkken i `main.py`, som orkestrerer systemet gjennom tilstandsmaskinen **HOME → SCAN → DETECT → PICK → PLACE**, inkludert retry-logikk, grip-verifikasjon og feilhåndtering på systemnivå.

#### Konfigurasjon og kalibrering
Jobbet med konfigurasjonsarkitekturen i `arm.py` (fysiske konstanter, bin-posisjoner, SCAN_POSE, HOME_POSITION, link-lengder, grab heights og sag-modell parametere). Bidro til `02_joints.py`, manuell Step 2b-prosedyre for klo-oppsett/validering, samt end-to-end pick-test (`08_pick_test.py`). Klo-oppsettet er Step 2b og gjøres manuelt ved å sentrere M5 eksternt i Dynamixel Wizard, montere den 3D-printede kloa åpen, lukke sakte til ønsket gripeposisjon, skrive verdiene til `CLAW_OPEN_POS` og `CLAW_CLOSED_POS`, og deretter validere adaptiv klo-/grip-adferd med `src/calibration/02b_claw_grip_test.py` som Step 2b-validering. Det finnes derfor ikke lenger et aktivt `src/calibration/02b_claw.py`-skript i kodebasen.

Utviklet også Step 10, `src/calibration/10_bin_calibration.py` — interaktivt bin-posisjon-kalibreringsverktøy med WASD-styring og limp mode for presis plassering. Lagrer kalibrerte bin-koordinater til JSON-fil som lastes dynamisk av systemet.

Implementerte sentrale funksjoner i `arm.py` for bin-håndtering: `load_bin_calibration()` for dynamisk lasting av kalibrerte bin-posisjoner, `get_bin_coords()` for oppslag, `get_bin_m4_offset()` for håndleddskorreksjoner, `compute_grab_height()` for distanse-basert grep-høyde-interpolasjon, og `compute_wrist_correction()` for håndleddskompensasjon. Implementerte også SORTING/DROPPING-states i `main.py` som bruker de kalibrerte bin-posisjonene for presist ball-avkast, samt grip-verifikasjon med last-måling.

Skrev `scripts/manual_tests/record_stats.py` for opptak av ytelses-statistikk (latens, FPS, syklustid) under kjøring, som grunnlag for `docs/performance.md`.

#### Hardware-løs testing
Satte opp `mock_serial.py` som simulerer seriellporten slik at `main.py` kan testes uten fysisk hardware tilkoblet.

### Uke 19 (4. mai – 10. mai): Raspberry Pi-integrasjon
Fokusert på integrasjon mot Raspberry Pi. Lastet GitHub-koden over på Pi-en, og jobbet med å sikre at programvaren kjører som forventet i det endelige miljøet.

### Hva som er planlagt fremover
*   **Full Raspberry Pi-integrasjon:** Fortsette samkjøring mellom Pi, OpenRB-150 og motorene.
*   **Ende-til-ende systemtest:** Forberede og gjennomføre full systemtest.
*   **Ferdigstilling av JSON-protokoll:** Fullføre implementeringen av JSON-kommunikasjonen.

---

## 5. Individuell Rapport: Filmon
**Rolle:** Utvikler (Bildedeteksjon/Maskinsyn)

### Uke 4–5 (20. januar – 1. februar): Oppstart og første prototyper
Deltok på oppstartsmøter, kartlegging med prosjektmodell, design og animasjon. Begynte arbeid med konseptprototype og programmering av modeller. Laget første modeller i forbindelse med ML på ulike nivåer, gjennomførte tester og analyser, simulerte sensor og ML-modeller, og utforsket prototype i samspill med RGB-bilde og ML-modeller. Forberedte 1. presentasjon (5. februar).

### Uke 6–11 (5. februar – 15. mars): Hovedfase 1 – Maskinsyn-utvikling
*   **Samarbeid om bildedeteksjon:** Jobbet tett sammen med Ole for å påbegynne og utforske løsninger for bildedeteksjon i Python.
*   **Innsamling av testdata:** Tok bilder av røde og blå kuler med ulike lysforhold som datagrunnlag.
*   **Teknisk utforsking:** Satte meg inn i OpenCV-biblioteket for å forstå hvordan vi best mulig kan filtrere ut farger og finne koordinatene til objektene.

### Uke 12–13 (16. mars – 29. mars): Modellforbedring og 2. presentasjon (26. mars)
Jobbet med regulering og normalisering av modellen for økt pålitelighet. Forberedte 2. presentasjon (26. mars).

### Uke 14–17 (30. mars – 26. april): Deteksjonsmodeller – HSV vs YOLO

I denne perioden har hovedfokuset mitt vært å utvikle og evaluere deteksjonsmodeller for robotarmens maskinsyn. Jeg har jobbet etter en iterativ end-to-end-prosess: **Collect Data → Label → Train → Test → Deploy → Observe Errors → Improve Data → Repeat**, og kjørt kontinuerlig trening for å sikre stabilitet.

#### Tre hovedoppgaver systemet må løse
1.  **Lokalisering:** Hvor ballene befinner seg – ikke bare hvilken farge, men også posisjonen. Viktig fordi samme farge kan opptre i ulike former (trekant, kuler, firkanter).
2.  **Deteksjon:** Tester alle objekter (baller, trekanter og rektangler), selv om kun baller er en del av systemets krav.
3.  **Klassifisering:** Identifisere både form (ball, trekant, rektangulær boks) og farge (blå, rød og grønn).

#### Alternativ 1: HSV-basert deteksjon
*   **Fordeler:** Enkel og rask, kan gjennomføres med grunnleggende bildebehandling (OpenCV). Krever mindre regnekraft og data, og er lett å justere fargesegmenteringen for.
*   **Ulemper:** Veldig sensitiv for lysstyrke og skygger, som påvirker segmenteringen.
*   **Datagrunnlag:** Tok 264 bilder under ulike forhold for å teste modellen.

#### Alternativ 2: YOLO (dyp læring)
*   Brukte over 600 bilder for å annotere baller med klasser samt trekanter.
*   **Fordeler:** Effektiv mot varierende lysforhold, bakgrunn og overlapping. Robust nok til å skille objekter med samme farge basert på form.
*   **Ulemper:** Modellen er datasulten. Trening krever ekstra tid og GPU. Vurdert som overkill for vårt enkle datasett.

#### Modellevaluering opp mot systemkrav
*   **YOLO** viser god progresjon på både posisjon og farge, fungerer godt på lav oppløsning (64x64 piksler).
*   **HSV-segmentering** fungerer best på 128x128 piksler. Maske brukes for å segmentere bestemte farger (rød og blå).
*   **Konklusjon:** HSV er bekreftet på deploy-nivå, mens YOLO ble testet på et datasett med 232 bilder for sammenligning.

### Uke 19 (4. mai – 10. mai): Rapportskriving
Hovedfokuset denne uken har vært rapportskriving. Jobbet med å dokumentere arbeidet rundt bildedeteksjonen og maskinsynet i hovedrapporten.

### Hva som er planlagt fremover
*   **Videre rapportarbeid:** Fortsette skrivingen og strukturere innholdet i tråd med resten av gruppens bidrag.
*   **Støtte til testing:** Bistå med eventuelle justeringer i deteksjonsmodellen ved behov.
*   **Maskinvareoppsett:** Hjelpe til med Raspberry Pi-konfigurasjon når aktuelt.
