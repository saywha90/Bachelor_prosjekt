# Test 1 – Overføring og oppstart av balldeteksjon på Raspberry Pi 5

Dato: 10. april 2026  
Mål: Overføre kode fra Mac til Raspberry Pi 5 og kjøre live balldeteksjon via VNC.

---

## Steg 1 – Finn IP-adressen til Raspberry Pi

**På Pi-en (via VNC-terminal):**
```bash
hostname -I
```
**Hva:** Viser alle IP-adresser Pi-en er tildelt på nettverket.  
**Hvorfor:** Vi trenger IP-adressen for å koble til Pi-en fra Macen med SSH og rsync.  
**Resultat:** `192.168.0.21`

---

## Steg 2 – Test SSH-tilkobling fra Mac

**På Macen:**
```bash
ssh pi@robotpi.local
```
**Hva:** SSH (Secure Shell) åpner en kryptert terminal-økt mot Pi-en.  
**Hvorfor:** Lar oss kjøre kommandoer på Pi-en direkte fra Mac-terminalen – uten skjerm eller tastatur på Pi.  
**Resultat:** OK – logget inn med passord.

---

## Steg 3 – Lag prosjektmappe på Pi-en

**På Pi-en (via SSH):**
```bash
mkdir -p ~/Bachelor_prosjekt/src
```
**Hva:** Oppretter mappen `Bachelor_prosjekt/src` i hjemmemappen til brukeren `pi`. `-p` betyr at eventuelle mellommapper også opprettes automatisk.  
**Hvorfor:** rsync (neste steg) krever at destinasjonsmappen finnes på forhånd.

---

## Steg 4 – Overfør kode fra Mac til Pi

**På Macen (ny terminal-fane):**
```bash
rsync -avz \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude 'venv_train' \
  --exclude '.venv' \
  "/Users/olhage14/Documents/Bachelor 2026 - USN/bachelor_2026_kode/Bachelor_prosjekt/src/" \
  pi@robotpi.local:~/Bachelor_prosjekt/src/
```
**Hva:** `rsync` er et verktøy for effektiv filoverføring.  
- `-a` = arkivmodus (bevarer filrettigheter og struktur)  
- `-v` = verbose (viser hvilke filer som overføres)  
- `-z` = komprimerer data under overføring  
- `--exclude` = hopper over mapper vi ikke trenger på Pi-en  

**Hvorfor:** Raskere enn å kopiere manuelt, og kopierer kun filer som har endret seg ved fremtidige overføringer.  
**Resultat:** 21 filer overført.

---

## Steg 5 – Sett opp virtuelt Python-miljø på Pi-en

**På Pi-en (via SSH):**
```bash
cd ~/Bachelor_prosjekt
python3 -m venv .venv
```
**Hva:** Oppretter et isolert Python-miljø i mappen `.venv`.  
**Hvorfor:** Hindrer at pakker vi installerer kolliderer med systemets Python-pakker. God praksis på alle plattformer.

---

## Steg 6 – Aktiver det virtuelle miljøet

**På Pi-en:**
```bash
source .venv/bin/activate
```
**Hva:** Aktiverer det virtuelle miljøet. Du ser `(.venv)` foran prompten når det er aktivt.  
**Hvorfor:** Alle `pip install`-kommandoer etter dette blir installert i `.venv`, ikke globalt.

---

## Steg 7 – Installer Python-pakker

**På Pi-en:**
```bash
pip install -r src/requirements.txt
```
**Hva:** Installerer alle pakker listet i `requirements.txt` (numpy, opencv, depthai, scikit-learn, osv.).  
**Hvorfor:** Koden bruker disse bibliotekene for bildebehandling og kameraopptak.

---

## Steg 8 – Fiksing av kameratillatelser (udev-regler)

**Feilmelding vi fikk:**
```
[depthai] [warning] Insufficient permissions to communicate with X_LINK_UNBOOTED device
```

**Løsning på Pi-en:**
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
**Hva:**  
- Første linje: Oppretter en udev-regel som gir alle brukere lesetilgang til OAK-kameraet (USB vendor ID `03e7` = Luxonis/Intel Movidius).  
- `udevadm control --reload-rules`: Laster inn reglene på nytt uten å restarte Pi-en.  
- `udevadm trigger`: Bruker reglene på allerede tilkoblede enheter.  

**Hvorfor:** Linux krever spesifikke tillatelser for å kommunisere med USB-enheter. Uten disse reglene kan vanlige brukere (og Python-programmer) ikke snakke med kameraet.  
**Etter:** Koble USB-kabelen ut og inn igjen, deretter fungerer kameraet.

---

## Steg 9 – Kjør balldeteksjon (via VNC)

**Åpne terminal på Pi-en via VNC og kjør:**
```bash
cd ~/Bachelor_prosjekt
source .venv/bin/activate
python src/vision/test_enhanced_detector.py
```
**Hva:** Starter live balldeteksjon med OAK-kameraet. Et vindu åpnes (1280×800) med:
- HUD-panel øverst til venstre med hvit tekst
- FPS og antall analyserte frames
- Rullende snitt-konfidens (stiger jo flere baller som detekteres)
- Lysnivå (300–700 lux)
- Antall baller detektert (rød/blå)
- Per-ball-info: Form-% og Farge-%  
- Sirkler rundt detekterte baller med label

**Kontroller i vinduet:**
- `q` – Avslutt
- `s` – Skriv ut statistikk i terminalen
- `r` – Nullstill statistikk

---

## Fremtidige oppdateringer – slik overfører du ny kode

Hver gang du endrer kode på Macen, kjør bare steg 4 på nytt:
```bash
rsync -avz \
  --exclude '__pycache__' --exclude '*.pyc' --exclude 'venv_train' --exclude '.venv' \
  "/Users/olhage14/Documents/Bachelor 2026 - USN/bachelor_2026_kode/Bachelor_prosjekt/src/" \
  pi@robotpi.local:~/Bachelor_prosjekt/src/
```
Deretter restart programmet på Pi-en.
