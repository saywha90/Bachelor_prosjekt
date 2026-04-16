/*
 * openrb_bridge.ino
 * =================
 * OpenRB-150 firmware that acts as a USB bridge between a Raspberry Pi
 * and 5 daisy-chained Dynamixel motors.
 *
 * Protocol
 * --------
 *   Pi  ──USB-C──►  OpenRB-150  ──TTL/RS485──►  Dynamixel chain
 *
 *   The Pi sends a JSON string terminated by '\n':
 *       {"m1":2048,"m2":1024,"m3":3000,"m4":2048,"m5":2048}\n
 *
 *   The bridge parses it and commands each motor to the goal position.
 *   It replies with "OK\n" on success or "ERR:<message>\n" on failure.
 *
 * Dependencies
 * ------------
 *   - Dynamixel2Arduino  (installable via Arduino Library Manager)
 *   - ArduinoJson v7+    (installable via Arduino Library Manager)
 *
 * Hardware
 * --------
 *   Motor 1 (XM430)  – Base Pan      – ID 1
 *   Motor 2 (XM540)  – Shoulder Tilt – ID 2
 *   Motor 3 (XM430)  – Elbow Tilt    – ID 3
 *   Motor 4 (XL430)  – Wrist Tilt    – ID 4
 *   Motor 5 (XL430)  – Claw          – ID 5
 */

#include <Dynamixel2Arduino.h>
#include <ArduinoJson.h>

// ── OpenRB-150 Dynamixel serial port ─────────────────────────────────
// On the OpenRB-150 the Dynamixel bus is exposed as Serial1 at the
// default baud rate of 57600.  Adjust if you've changed the motor baud.
#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1          // OpenRB-150 handles direction automatically

const uint32_t DXL_BAUDRATE = 57600;

// ── USB serial to the Raspberry Pi ───────────────────────────────────
#define PI_SERIAL    Serial
const uint32_t PI_BAUDRATE = 115200;

// ── Motor IDs ────────────────────────────────────────────────────────
const uint8_t MOTOR_IDS[]  = {1, 2, 3, 4, 5};
const uint8_t NUM_MOTORS   = 5;

// ── Motion profile (tune these for smooth, safe movement) ────────────
//    Profile Velocity:      units = 0.229 rev/min  (0 = max speed)
//    Profile Acceleration:  units = 214.577 rev/min²
//
//    Conservative defaults below ≈ gentle, non-jerky motion.
const uint32_t PROFILE_VELOCITY     = 80;   // ~18 RPM – slow & safe
const uint32_t PROFILE_ACCELERATION = 20;   // gentle ramp

// ── Dynamixel control-table addresses (Protocol 2.0) ─────────────────
//    These are identical across XM540, XM430, and XL430.
const uint16_t ADDR_PROFILE_ACCELERATION = 108;
const uint16_t ADDR_PROFILE_VELOCITY     = 112;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Using the DYNAMIXEL namespace for unit types
using namespace ControlTableItem;

// ── JSON receive buffer ──────────────────────────────────────────────
const size_t JSON_BUF_SIZE = 256;
char jsonBuffer[JSON_BUF_SIZE];
size_t bufIndex = 0;

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
    // USB serial to Pi
    PI_SERIAL.begin(PI_BAUDRATE);
    while (!PI_SERIAL);   // wait for USB enumeration

    // Dynamixel bus
    dxl.begin(DXL_BAUDRATE);
    dxl.setPortProtocolVersion(2.0);

    // ── Initialise each motor ────────────────────────────────────────
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        uint8_t id = MOTOR_IDS[i];

        // Ping to verify the motor is on the bus
        if (!dxl.ping(id)) {
            PI_SERIAL.print("ERR:Motor ");
            PI_SERIAL.print(id);
            PI_SERIAL.println(" not found");
            continue;
        }

        // Ensure we're in Position Control mode (mode 3)
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, OP_POSITION);

        // Set smooth motion profile
        dxl.writeControlTableItem(PROFILE_ACCELERATION, id, PROFILE_ACCELERATION);
        dxl.writeControlTableItem(PROFILE_VELOCITY, id, PROFILE_VELOCITY);

        // Enable torque
        dxl.torqueOn(id);
    }

    PI_SERIAL.println("OK:READY");
}

// =====================================================================
//  LOOP – listen for JSON commands and drive motors
// =====================================================================
void loop() {
    while (PI_SERIAL.available()) {
        char c = (char)PI_SERIAL.read();

        if (c == '\n' || c == '\r') {
            if (bufIndex > 0) {
                jsonBuffer[bufIndex] = '\0';
                processCommand(jsonBuffer);
                bufIndex = 0;
            }
        } else {
            if (bufIndex < JSON_BUF_SIZE - 1) {
                jsonBuffer[bufIndex++] = c;
            } else {
                // Overflow – discard and reset
                bufIndex = 0;
                PI_SERIAL.println("ERR:JSON too long");
            }
        }
    }
}

// =====================================================================
//  Parse JSON and command the motors
// =====================================================================
void processCommand(const char* json) {
    // ArduinoJson v7 – StaticJsonDocument replaced by JsonDocument
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err) {
        PI_SERIAL.print("ERR:JSON parse failed: ");
        PI_SERIAL.println(err.c_str());
        return;
    }

    // Validate that all five keys are present
    if (!doc.containsKey("m1") || !doc.containsKey("m2") ||
        !doc.containsKey("m3") || !doc.containsKey("m4") ||
        !doc.containsKey("m5")) {
        PI_SERIAL.println("ERR:Missing motor key(s)");
        return;
    }

    int32_t positions[NUM_MOTORS];
    positions[0] = doc["m1"].as<int32_t>();
    positions[1] = doc["m2"].as<int32_t>();
    positions[2] = doc["m3"].as<int32_t>();
    positions[3] = doc["m4"].as<int32_t>();
    positions[4] = doc["m5"].as<int32_t>();

    // Range-check each position (0 – 4095)
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (positions[i] < 0 || positions[i] > 4095) {
            PI_SERIAL.print("ERR:m");
            PI_SERIAL.print(i + 1);
            PI_SERIAL.print(" out of range: ");
            PI_SERIAL.println(positions[i]);
            return;
        }
    }

    // ── Command motors ──────────────────────────────────────────────
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        dxl.setGoalPosition(MOTOR_IDS[i], (uint32_t)positions[i]);
    }

    // Acknowledge success
    PI_SERIAL.println("OK");
}
