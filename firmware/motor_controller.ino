/**
 * motor_controller.ino
 * 
 * Firmware for Arduino Mega som styrer robotarmen.
 * Bruker FreeRTOS for å separere kommunikasjon og motorstyring i to uavhengige tråder (tasks).
 * 
 * VIKTIG:
 * Denne koden er designet for å være agnostisk til antall motorer.
 * For å oppgradere fra 3 til 6 akser, endre `NUM_JOINTS` nedenfor.
 */

#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <semphr.h>  // For å beskytte delte variabler (mutex)

// ==========================================
// KONFIGURASJON
// ==========================================

// Antall ledd (MÅ matche config.py på Raspberry Pi!)
#define NUM_JOINTS 3

// Pins for servoene.
// Hvis vi oppgraderer til 6 akser, fyller vi bare ut resten av listen.
const int SERVO_PINS[NUM_JOINTS] = {2, 3, 4}; // Eks: Base=2, Skulder=3, Albue=4

// Protokoll-konstanter
const byte START_BYTE = 0xFF;
const byte END_BYTE = 0xFE;

// ==========================================
// GLOBALE VARIABLER
// ==========================================

Servo servos[NUM_JOINTS]; // Array av servo-objekter

// Delt buffer for mål-vinkler som mottas fra Seriell.
// Denne leses av Control-tasken og skrives til av Serial-tasken.
int targetAngles[NUM_JOINTS];

// Mutex for å hindre at vi leser vinkler mens de oppdateres.
// Dette hindrer "tearing" (f.eks. at vi leser halvparten av en gammel pakke og halvparten av en ny).
SemaphoreHandle_t xAnglesMutex;

// ==========================================
// TASKS (Tråder)
// ==========================================

/**
 * TaskSerial:
 * Ansvarlig for å lytte til seriellporten, parse pakker og verifisere data.
 * Kjører med høy prioritet for ikke å miste data.
 */
void TaskSerial(void *pvParameters) {
  (void) pvParameters;

  // Buffer for å lagre innkommende pakke midlertidig
  // Størrelse: Start + Count + Vinkler + CRC + End
  const int PACKET_SIZE = 1 + 1 + NUM_JOINTS + 1 + 1;
  byte buffer[PACKET_SIZE];
  
  for (;;) {
    // Sjekk om vi har nok data i bufferet til en hel pakke
    if (Serial.available() >= PACKET_SIZE) {
      
      // 1. Sjekk Start Byte
      if (Serial.peek() == START_BYTE) {
        
        // Les hele pakken inn i bufferet
        Serial.readBytes(buffer, PACKET_SIZE);
        
        // 2. Verifiser struktur
        byte count = buffer[1];
        byte receivedChecksum = buffer[PACKET_SIZE - 2];
        byte receivedEndByte = buffer[PACKET_SIZE - 1];

        if (count != NUM_JOINTS) {
            // Feil antall motorer i pakken. Ignorer.
            // (Her kan vi ev. flushe bufferet eller sende en feilmelding tilbake)
        } 
        else if (receivedEndByte != END_BYTE) {
            // Pakken sluttet ikke der vi forventet. Synkroniseringsfeil.
        }
        else {
            // 3. Beregn checksum (CRC)
            // Summen av alle bytes unntatt den siste (END_BYTE) og CRC-byten selv.
            // Må matche logikken i Python (sum % 256).
            int sum = 0;
            // Summer opp Start + Count + Data
            for (int i = 0; i < (PACKET_SIZE - 2); i++) {
                sum += buffer[i];
            }
            byte calculatedChecksum = sum % 256;

            if (calculatedChecksum == receivedChecksum) {
                // PAKKEN ER GYLDIG! Oppdater målvinklene.
                
                // Ta låsen før vi skriver til de delte variablene
                if (xSemaphoreTake(xAnglesMutex, (TickType_t) 10) == pdTRUE) {
                    for (int i = 0; i < NUM_JOINTS; i++) {
                        // Vinklene starter på index 2 i bufferet (etter Start og Count)
                        targetAngles[i] = buffer[2 + i];
                    }
                    xSemaphoreGive(xAnglesMutex); // Slipp låsen
                }
            } else {
                // Checksum feilet. Data kan være korrupt.
                // Serial.println("CRC Error"); 
            }
        }
      } else {
        // Start-byte ikke funnet. Kast denne byten og prøv neste.
        // Dette hjelper oss å "finne takten" igjen hvis vi kom ut av sync.
        Serial.read(); 
      }
    }
    
    // La andre tasks slippe til. 10ms pause er nok til å sjekke 100 ganger i sekundet.
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

/**
 * TaskControl:
 * Ansvarlig for å fysisk flytte servoene.
 * Kan utvides til å inkludere bevegelsesprofiler (smooth acceleration) senere.
 */
void TaskControl(void *pvParameters) {
  (void) pvParameters;

  int currentAngles[NUM_JOINTS];

  // Initialiser lokale variabler
  for(int i=0; i<NUM_JOINTS; i++) currentAngles[i] = 90;

  for (;;) {
    
    // Hent de siste ønskede posisjonene
    if (xSemaphoreTake(xAnglesMutex, (TickType_t) 10) == pdTRUE) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            currentAngles[i] = targetAngles[i];
        }
        xSemaphoreGive(xAnglesMutex);
    }

    // Oppdater servoene
    for (int i = 0; i < NUM_JOINTS; i++) {
        // Her kan vi legge til logikk for "Mapping":
        // Hvis robotens "0 grader" ikke er servoens "0 grader", fikser vi det her.
        int servoVal = mapLogicToServo(i, currentAngles[i]);
        
        servos[i].write(servoVal);
    }

    // Servoer trenger ca 20ms per oppdatering (50Hz PWM).
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// HJELPEFUNKSJONER
// ==========================================

/**
 * Oversetter fra robotens logiske vinkel til servoens fysiske vinkel.
 * Nyttig hvis servoene er montert opp-ned eller med en offset.
 */
int mapLogicToServo(int jointIndex, int logicAngle) {
    // Eksempel: Ledd 1 (Skulder) er montert "speilvendt" slik at 0 er 180.
    /*
    if (jointIndex == 1) {
        return 180 - logicAngle;
    }
    */
    
    // For Fase 1 antar vi 1:1 mapping.
    // Vi klemmer også verdien mellom 0 og 180 for sikkerhets skyld.
    if (logicAngle < 0) return 0;
    if (logicAngle > 180) return 180;
    return logicAngle;
}

// ==========================================
// SETUP & LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  
  // Opprett Mutex
  xAnglesMutex = xSemaphoreCreateMutex();

  // Konfigurer Servoer og sett startposisjon
  if (xAnglesMutex != NULL) {
      for (int i = 0; i < NUM_JOINTS; i++) {
        servos[i].attach(SERVO_PINS[i]);
        targetAngles[i] = 90; // Start i midtstilling (trygt)
        servos[i].write(90);
      }
  }

  // Opprett Tasks
  // Stack size 128 ord er vanligvis nok for enkle oppgaver på AVR.
  xTaskCreate(TaskSerial, "Serial", 256, NULL, 2, NULL); // Høyere prioritet (2)
  xTaskCreate(TaskControl, "Control", 128, NULL, 1, NULL); // Lavere prioritet (1)

  // Start planleggeren (Scheduler). Denne funksjonen returnerer aldri.
  vTaskStartScheduler();
}

void loop() {
  // Tom. Alt skjer i tasks når man bruker FreeRTOS.
}
