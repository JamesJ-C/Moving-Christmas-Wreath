#include <Arduino.h>
#include "Servo42C.h"

Servo42C motor;

// Choose which ESP32 pins you wired to the motor’s UART
static const int SERVO_RX_PIN = 16;  // ESP32 RX (goes to motor TX)
static const int SERVO_TX_PIN = 17;  // ESP32 TX (goes to motor RX)

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("MKS SERVO42C demo starting...");

    // Init UART to motor
    Serial2.begin(38400, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
    motor.begin(Serial2, 38400, 0xE0, 100);  // addr 0xE0 is the default

    if (!motor.enable(true)) {
        Serial.println("Failed to enable motor!");
    } else {
        Serial.println("Motor enabled.");
    }

    // Run CW at a modest speed
    if (!motor.runConstantSpeed(Servo42C::CW, 20)) {
        Serial.println("Failed to start constant-speed run");
    } else {
        Serial.println("Motor running CW at speed 20");
    }

    delay(2000);

    Serial.println("Stopping motor...");
    motor.stop();

    // // Example: move exactly 1 rev (for 200-step motor with 16 microsteps = 3200 pulses)
    uint32_t oneRevPulses = 3200;
    Serial.println("Running 1 revolution CCW...");
    if (!motor.runSteps(Servo42C::CCW, 10, oneRevPulses)) {
        Serial.println("runSteps failed");
    } else {
        Serial.println("runSteps command accepted");
    }

        delay(2000);

    Serial.println("Stopping motor...");
    motor.stop();
    delay(10000);
}

void loop() {


//  motor.runConstantSpeed(Servo42C::CW, 127);


  //   // Periodically check shaft error for debugging
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        uint16_t err;
        if (motor.readError(err)) {
            Serial.print("Shaft error = ");
            Serial.println(err);
        } else {
            Serial.println("Failed to read shaft error");
        }
    }
}
