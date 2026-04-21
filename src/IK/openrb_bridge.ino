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

const uint32_t DXL_BAUDRATE = 115200;

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
const uint32_t PROF_VEL_VALUE  = 80;   // ~18 RPM – slow & safe
const uint32_t PROF_ACC_VALUE  = 20;   // gentle ramp

// ── Startup-specific slower profile ──────────────────────────────────
const uint32_t STARTUP_PROF_VEL  = 30;   // slower for startup
const uint32_t STARTUP_PROF_ACC  = 10;   // gentler acceleration for startup

// ── Dynamixel control-table addresses (Protocol 2.0) ─────────────────
//    These are identical across XM540, XM430, and XL430.
const uint16_t ADDR_PROFILE_ACCELERATION = 108;
const uint16_t ADDR_PROFILE_VELOCITY     = 112;
const uint16_t ADDR_HARDWARE_ERROR       = 70;   // Hardware Error Status

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

        // ── Clear any latched hardware errors by rebooting the motor ──
        //    Dynamixel motors latch errors (overload, position limit, etc.)
        //    in the Hardware Error Status register (addr 70).  Once set,
        //    the LED blinks red and the motor refuses to move until the
        //    error is cleared.  The ONLY way to clear it via Protocol 2.0
        //    is to send a Reboot instruction.
        uint8_t hw_err = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id);
        if (hw_err != 0) {
            PI_SERIAL.print("WARN:Motor ");
            PI_SERIAL.print(id);
            PI_SERIAL.print(" has hardware error 0x");
            PI_SERIAL.print(hw_err, HEX);
            PI_SERIAL.println(" — rebooting to clear");
            dxl.reboot(id);
            delay(500);  // wait for motor to finish rebooting
            // Re-ping after reboot
            if (!dxl.ping(id)) {
                PI_SERIAL.print("ERR:Motor ");
                PI_SERIAL.print(id);
                PI_SERIAL.println(" not responding after reboot");
                continue;
            }
        }

        // Ensure we're in Position Control mode (mode 3)
        dxl.torqueOff(id);
        dxl.setOperatingMode(id, OP_POSITION);

        // Set smooth motion profile
        dxl.writeControlTableItem(PROFILE_ACCELERATION, id, PROF_ACC_VALUE);
        dxl.writeControlTableItem(PROFILE_VELOCITY, id, PROF_VEL_VALUE);

        // Enable torque
        dxl.torqueOn(id);
        delay(50);  // let the motor PID settle
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

    // ── Handle "read_pos" command ────────────────────────────────────
    if (doc.containsKey("cmd")) {
        const char* cmd = doc["cmd"].as<const char*>();

        if (strcmp(cmd, "read_pos") == 0) {
            // Read current positions of all 5 motors and send back as JSON
            JsonDocument resp;
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                char key[4];
                snprintf(key, sizeof(key), "m%d", MOTOR_IDS[i]);
                resp[key] = (int32_t)dxl.getPresentPosition(MOTOR_IDS[i]);
            }
            serializeJson(resp, PI_SERIAL);
            PI_SERIAL.println();
            return;
        }

        if (strcmp(cmd, "read_errors") == 0) {
            // Read Hardware Error Status of all 5 motors and send back as JSON
            JsonDocument resp;
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                char key[4];
                snprintf(key, sizeof(key), "m%d", MOTOR_IDS[i]);
                resp[key] = (uint8_t)dxl.readControlTableItem(HARDWARE_ERROR_STATUS, MOTOR_IDS[i]);
            }
            serializeJson(resp, PI_SERIAL);
            PI_SERIAL.println();
            return;
        }

        if (strcmp(cmd, "set_profile") == 0) {
            // Set motion profile on all motors
            uint32_t vel = doc["vel"].as<uint32_t>();
            uint32_t acc = doc["acc"].as<uint32_t>();
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                uint8_t id = MOTOR_IDS[i];
                dxl.writeControlTableItem(PROFILE_ACCELERATION, id, acc);
                dxl.writeControlTableItem(PROFILE_VELOCITY, id, vel);
            }
            PI_SERIAL.println("{\"status\":\"profile_set\"}");
            return;
        }

        if (strcmp(cmd, "enable_torque") == 0) {
            // Enable torque on all motors (useful if power cycled)
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                dxl.torqueOn(MOTOR_IDS[i]);
            }
            PI_SERIAL.println("OK:TORQUE_ON");
            return;
        }

        if (strcmp(cmd, "clear_errors") == 0) {
            // Reboot any motor that has a latched hardware error, then
            // re-configure it (position mode, profile, torque on).
            // This clears the red-blinking LED and allows the motor to move again.
            uint8_t cleared = 0;
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                uint8_t id = MOTOR_IDS[i];
                uint8_t hw_err = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id);
                if (hw_err != 0) {
                    dxl.reboot(id);
                    delay(500);
                    if (dxl.ping(id)) {
                        dxl.torqueOff(id);
                        dxl.setOperatingMode(id, OP_POSITION);
                        dxl.writeControlTableItem(PROFILE_ACCELERATION, id, PROF_ACC_VALUE);
                        dxl.writeControlTableItem(PROFILE_VELOCITY, id, PROF_VEL_VALUE);
                        dxl.torqueOn(id);
                        cleared++;
                    }
                }
            }
            PI_SERIAL.print("{\"cleared\":");
            PI_SERIAL.print(cleared);
            PI_SERIAL.println("}");
            return;
        }

        if (strcmp(cmd, "diagnose") == 0) {
            // Comprehensive motor diagnostics across multiple baud rates
            const uint32_t baudRates[] = {57600, 115200, 1000000};
            const uint8_t numBauds = 3;

            PI_SERIAL.print("{\"diagnostics\":[");

            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                uint8_t id = MOTOR_IDS[i];
                if (i > 0) PI_SERIAL.print(",");
                PI_SERIAL.print("{\"id\":");
                PI_SERIAL.print(id);

                bool found = false;
                uint32_t foundBaud = 0;
                int32_t pos = 0;
                uint16_t model = 0;

                // Try each baud rate
                for (uint8_t b = 0; b < numBauds; b++) {
                    dxl.begin(baudRates[b]);
                    dxl.setPortProtocolVersion(2.0);
                    delay(50);

                    if (dxl.ping(id)) {
                        found = true;
                        foundBaud = baudRates[b];
                        pos = (int32_t)dxl.getPresentPosition(id);
                        model = dxl.getModelNumber(id);
                        break;
                    }
                }

                PI_SERIAL.print(",\"found\":");
                PI_SERIAL.print(found ? "true" : "false");
                if (found) {
                    PI_SERIAL.print(",\"baud\":");
                    PI_SERIAL.print(foundBaud);
                    PI_SERIAL.print(",\"position\":");
                    PI_SERIAL.print(pos);
                    PI_SERIAL.print(",\"model\":");
                    PI_SERIAL.print(model);
                }
                PI_SERIAL.print("}");
            }

            PI_SERIAL.println("]}");

            // Restore original baud rate
            dxl.begin(DXL_BAUDRATE);
            dxl.setPortProtocolVersion(2.0);
            return;
        }

        // Unknown command
        PI_SERIAL.print("ERR:Unknown cmd: ");
        PI_SERIAL.println(cmd);
        return;
    }

    // ── Handle motor goal positions ──────────────────────────────────
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
