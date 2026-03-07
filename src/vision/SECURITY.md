# 🔒 SIKKERHETS- OG PERSONVERNSDOKUMENTASJON

Dette dokumentet beskriver sikkerhetstiltak og personvernhensyn i balldeteksjonssystemet.

---

## 📋 INNHOLDSFORTEGNELSE

1. [Sikkerhetsanalyse](#sikkerhetsanalyse)
2. [GDPR-Compliance](#gdpr-compliance)
3. [Implementerte Sikkerhetstiltak](#implementerte-sikkerhetstiltak)
4. [Anbefalte Praksiser](#anbefalte-praksiser)
5. [Sikkerhetstekniske Begrensninger](#sikkerhetstekniske-begrensninger)
6. [Rapportering av Sårbarheter](#rapportering-av-s%C3%A5rbarheter)

---

## 🔍 SIKKERHETSANALYSE

### ✅ POSITIVE FUNN

**1. Ingen Hardkodede Hemmeligheter**
- ✓ Ingen passord, API-nøkler eller tokens i koden
- ✓ Ingen databaseforbindelser eller credentials
- ✓ All konfigurasjon er åpen og ikke-sensitiv

**2. Minimal Datalagring**
- ✓ Ingen lagring av kamerabilder til disk
- ✓ Ingen logging av personlig identifiserbar informasjon
- ✓ Kun tekniske kalibreringsdata (HSV-verdier) lagres lokalt
- ✓ All data er midlertidig og slettes ved programavslutning

**3. Lokal Behandling**
- ✓ Ingen nettverkskommunikasjon
- ✓ Ingen data sendes til eksterne servere
- ✓ All prosessering skjer lokalt på enheten

**4. God Ressurshåndtering**
- ✓ Kamera frigis korrekt i `finally`-blokker
- ✓ Vinduer lukkes ved avslutning
- ✓ Ingen kjente memory leaks

**5. Ingen Dangerous Operations**
- ✓ Ingen SQL-operasjoner (ingen SQL injection-risiko)
- ✓ Ingen shell-kommandoer (ingen command injection-risiko)
- ✓ Ingen eval() eller exec() kall
- ✓ Ingen deserialisering av untrusted data

---

## 🛡️ GDPR-COMPLIANCE

### Personvernerklæring

**Databehandling:**
- Systemet prosesserer live video fra kamera
- Ingen bilder eller video lagres permanent
- Kun ballposisjoner, farger og tekniske parametere ekstraheres
- Ingen persondata samles inn eller lagres

**Rettslig Grunnlag:**
- Samtykke innhentes før kamerabruk
- Brukeren kan når som helst trekke tilbake samtykket ved å avslutte programmet

**Brukerrettigheter:**
- Rett til å nekte kamerabruk
- Rett til å avslutte prosessering når som helst
- Rett til innsyn (ingen data lagres å se på)
- Rett til sletting (all data slettes automatisk ved avslutning)

**Datalagring:**
- **Tekniske data**: HSV-kalibreringsdata lagres i `hsv_calibration.txt` (ikke personlig)
- **Statistikk**: Antall deteksjoner per farge (ikke personlig)
- **Video/bilder**: IKKE lagret
- **Lokasjonsdata**: IKKE samlet
- **Personidentifiserende info**: IKKE samlet

**Tredjepartsdeling:**
- Ingen data deles med tredjeparter
- Ingen cloud-tjenester brukes
- Alt er 100% lokalt

---

## 🔐 IMPLEMENTERTE SIKKERHETSTILTAK

### 1. Samtykke-Mekanisme (privacy_utils.py)

```python
def request_camera_consent(auto_consent: bool = False) -> bool:
    """
    Ber om eksplisitt brukersamtykke før kamerabruk.
    Implementerer GDPR artikkel 6(1)(a) - samtykke.
    """
```

**Hvor det brukes:**
- `test_ball_detection.py` - Før kamera åpnes
- `hsv_tuner.py` - Før kamera åpnes
- `ball_detection.py` - I testmodus

### 2. Input-Validering (privacy_utils.py)

```python
def get_validated_float_input(prompt, min_value, max_value, allow_cancel):
    """
    Validerer og saniterer brukerinput.
    Beskytter mot: ValueError, out-of-range, injection
    """
```

**Beskyttelse mot:**
- ❌ Ugyldige datatyper (ValueError)
- ❌ Out-of-range verdier (logiske feil)
- ❌ Injection attacks (ingen shell-kommandoer)
- ❌ DoS via ekstreme verdier

### 3. Resource Exhaustion Protection (ball_detection.py)

```python
class BallDetector:
    def __init__(self, ..., max_detections_per_frame: int = 50):
        """Begrenser antall deteksjoner for å forhindre minne-exhaust"""
```

**Beskyttelse mot:**
- ❌ Memory exhaustion ved ekstreme deteksjoner
- ❌ CPU-overbelastning
- ❌ Denial of Service (DoS)

### 4. Sikker Filhåndtering (privacy_utils.py)

```python
def safe_file_path(filename: str, allowed_extensions: list) -> bool:
    """
    Validerer filnavn mot path traversal og farlige operasjoner.
    """
```

**Beskyttelse mot:**
- ❌ Path traversal (`../../../etc/passwd`)
- ❌ Absolutte stier (`C:\Windows\System32\...`)
- ❌ Farlige filtyper
- ❌ Injeksjon av kommandoer

### 5. Error Sanitization

**Før (usikkert):**
```python
except Exception as e:
    print(f"FEIL: {e}")  # Kan lekke systeminfo
```

**Etter (sikkert):**
```python
except Exception as e:
    print("❌ FEIL: En uventet feil oppstod")
    logging.error(f"Detaljert feil: {e}")  # Log internt, ikke til bruker
```

### 6. Kameraindeks-Validering

```python
def validate_camera_index(camera_index: int) -> bool:
    """Validerer at kameraindeks er innenfor rimelig rekkevidde"""
    return 0 <= camera_index <= 10
```

---

## 📚 ANBEFALTE PRAKSISER

### For Utvikling

**1. Aldri hardkod hemmeligheter**
```python
# ❌ DÅRLIG
api_key = "sk-1234567890abcdef"

# ✓ BRA
api_key = os.getenv("API_KEY")
```

**2. Valider all brukerinput**
```python
# Bruk alltid privacy_utils-funksjonene
from vision.privacy_utils import get_validated_float_input
value = get_validated_float_input("Enter value", min_value=0, max_value=100)
```

**3. Håndter ressurser korrekt**
```python
# Bruk alltid try-finally eller context managers
try:
    cap = cv2.VideoCapture(0)
    # ... kode ...
finally:
    cap.release()
```

**4. Begrens ressursbruk**
```python
# Sett alltid grenser for å forhindre DoS
detector = BallDetector(max_detections_per_frame=50)
```

### For Produksjon

**1. Enable Logging**
```python
import logging
logging.basicConfig(level=logging.INFO, filename='app.log')
```

**2. Regelmessige Sikkerhetsupdates**
```bash
pip install --upgrade opencv-python numpy
```

**3. Kjør Med Minste Privilegier**
- Ikke kjør som administrator/root
- Bruk dedikert bruker for kameraaksess

**4. Monitorer Ressursbruk**
```python
import psutil
memory_percent = psutil.virtual_memory().percent
if memory_percent > 90:
    print("ADVARSEL: Høyt minneforbruk!")
```

---

## ⚠️ SIKKERHETSTEKNISKE BEGRENSNINGER

### 1. Fysisk Sikkerhet
**Begrensning:** Systemet kan ikke beskytte mot fysisk manipulering av kamera
**Risiko:** Lav (robotarm-prosjekt, kontrollert miljø)
**Mitigering:** Bruk i sikret laboratorium/miljø

### 2. Kamera-Hijacking
**Begrensning:** Hvis andre prosesser får kameratilgang, kan de se video
**Risiko:** Medium (avhenger av OS-sikkerhet)
**Mitigering:** 
- Bruk OS-level kameratilgangskontroller
- Lukk programmet når ikke i bruk
- Indikatorlys på kamera viser når det er aktivt

### 3. Side-Channel Attacks
**Begrensning:** Systemet beskytter ikke mot timing attacks eller power analysis
**Risiko:** Ekstremt lav (ikke relevant for dette brukstilfellet)

### 4. Dependency Vulnerabilities
**Begrensning:** OpenCV og NumPy kan ha sårbarheter
**Risiko:** Medium
**Mitigering:**
```bash
# Sjekk for kjente sårbarheter
pip install safety
safety check
```

---

## 🔎 KJENTE RISIKOER OG MITIGERING

| Risiko | Alvorlighet | Sannsynlighet | Mitigering |
|--------|-------------|---------------|------------|
| Kamera-hijacking | Medium | Lav | OS-tilgangskontroll |
| Memory exhaustion | Medium | Lav | Max detections limitt |
| Path traversal | Lav | Ekstremt lav | Input validering |
| Error disclosure | Lav | Medium | Error sanitization |
| Dependency vuln. | Medium | Medium | Regelmessige updates |

---

## 🐛 RAPPORTERING AV SÅRBARHETER

Hvis du oppdager en sikkerhetssårbarhet:

**IKKE:**
- ❌ Del sårbarheter offentlig før de er fikset
- ❌ Utnytt sårbarheter

**GJØR:**
- ✓ Kontakt prosjektteamet direkte
- ✓ Gi detaljert beskrivelse av sårbarheten
- ✓ Foreslå en løsning hvis mulig
- ✓ Gi rimelig tid for reparasjon

**Informasjon å inkludere:**
1. Beskrivelse av sårbarheten
2. Steg for å reprodusere
3. Potensielt omfang/impact
4. Foreslått løsning
5. Proof-of-concept (hvis relevant)

---

## ✅ SIKKERHETS-SJEKKLISTE

For nye utviklere/brukere:

- [ ] Har lest denne sikkerhetsdokumentasjonen
- [ ] Forstår GDPR-implikasjoner
- [ ] Vet hvordan man bruker privacy_utils-funksjonene
- [ ] Kjører systemet med minste privilegier
- [ ] Holder avhengigheter oppdatert
- [ ] Logger sensitiv info IKKE til console
- [ ] Tester input-validering
- [ ] Lukker kamera når ikke i bruk
- [ ] Bruker try-finally for ressurshåndtering
- [ ] Har satt max_detections_per_frame

---

## 📖 REFERANSER

**GDPR:**
- [EU GDPR Official Text](https://gdpr-info.eu/)
- Artikkel 6(1)(a) - Samtykke som rettslig grunnlag
- Artikkel 13 - Informasjon til registrerte

**Sikkerhet Best Practices:**
- [OWASP Top 10](https://owasp.org/www-project-top-ten/)
- [Python Security Guide](https://python.readthedocs.io/en/stable/library/security_warnings.html)
- [OpenCV Security](https://docs.opencv.org/)

**Dependency Security:**
```bash
# Sjekk for sårbarheter
pip install pip-audit
pip-audit
```

---

**Sist oppdatert:** Mars 2026  
**Versjon:** 1.0  
**Prosjekt:** Bachelor Prosjekt 2026 - Autonomia

---

💡 **TIPS:** Kjør `python src/vision/privacy_utils.py` for å teste sikkerhetsfunksjonene!
