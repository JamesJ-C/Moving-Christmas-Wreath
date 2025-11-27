#pragma once
#include <Arduino.h>

class Servo42C {
public:
    enum Direction : uint8_t {
        CW  = 0,  // clockwise
        CCW = 1   // counter-clockwise
    };

    Servo42C()
        : _serial(nullptr), _addr(0xE0), _timeoutMs(50) {}

    void begin(HardwareSerial& serial,
               uint32_t baud = 38400,
               uint8_t addr = 0xE0,
               uint32_t timeoutMs = 50);

    // ---- Basic control ----
    bool enable(bool on);
    
    /**
    * Runs the speed either clockwise or counterclockwise at a speed
    * @param dir Runs the motor in either the Servo42C::CW, or Servo42C::CCW direction
    * @param speed mechanical? speed of the motor
    *
    */                                     // turn motor on/off
    bool runConstantSpeed(Direction dir, uint8_t speed);      // run at speed (0–127)
    bool stop();                                              // stop motor

    // ---- Step move ----
    bool runSteps(Direction dir, uint8_t speed, uint32_t pulses);

    // ---- Optional status ----
    bool readError(uint16_t& err);

private:
    HardwareSerial* _serial;
    uint8_t  _addr;
    uint32_t _timeoutMs;

    static uint8_t checksum8(const uint8_t* buf, size_t len);

    bool writeCommand(const uint8_t* buf, size_t len);
    bool readBytes(uint8_t* buf, size_t len);

    bool writeSimpleCmd(uint8_t code);
    bool writeCmd1Byte(uint8_t code, uint8_t data);
};
