#include "Servo42C.h"

// ---------- Internal helpers ----------

uint8_t Servo42C::checksum8(const uint8_t* buf, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void Servo42C::begin(HardwareSerial& serial,
                     uint32_t baud,
                     uint8_t addr,
                     uint32_t timeoutMs)
{
    _serial    = &serial;
    _addr      = addr;
    _timeoutMs = timeoutMs;

    _serial->begin(baud);
}

bool Servo42C::writeCommand(const uint8_t* buf, size_t len) {
    if (!_serial) return false;
    _serial->flush();
    _serial->write(buf, len);
    _serial->flush();
    return true;
}

bool Servo42C::readBytes(uint8_t* buf, size_t len) {
    if (!_serial) return false;

    uint32_t start = millis();
    size_t   idx   = 0;

    while (idx < len) {
        if (_serial->available()) {
            uint8_t b = (uint8_t)_serial->read();

            // Keep in sync: first byte must be the slave address.
            if (idx == 0 && b != _addr) {
                // Uncomment this if you want to see the junk:
                // Serial.print("Dropping byte 0x");
                // Serial.println(b, HEX);
                continue;  // skip garbage
            }

            buf[idx++] = b;

            // Debug (you can keep or remove):
            // Serial.print("RX[");
            // Serial.print(idx - 1);
            // Serial.print("] = 0x");
            // if (b < 0x10) Serial.print('0');
            // Serial.println(b, HEX);
        } else if (millis() - start > _timeoutMs) {
            // Serial.println("readBytes timeout");
            return false;
        }
    }
    return true;
}


bool Servo42C::writeSimpleCmd(uint8_t code) {
    uint8_t frame[3];
    frame[0] = _addr;
    frame[1] = code;
    frame[2] = checksum8(frame, 2);

    if (!writeCommand(frame, sizeof(frame))) return false;

    uint8_t resp[3];
    if (!readBytes(resp, sizeof(resp))) return false;

    if (resp[0] != _addr) return false;
    uint8_t crc = checksum8(resp, 2);
    if (crc != resp[2])   return false;

    return resp[1] == 1; // status OK
}

bool Servo42C::writeCmd1Byte(uint8_t code, uint8_t data) {
    uint8_t frame[4];
    frame[0] = _addr;
    frame[1] = code;
    frame[2] = data;
    frame[3] = checksum8(frame, 3);

    if (!writeCommand(frame, sizeof(frame))) return false;

    uint8_t resp[3];
    if (!readBytes(resp, sizeof(resp))) return false;

    if (resp[0] != _addr) return false;
    uint8_t crc = checksum8(resp, 2);
    if (crc != resp[2])   return false;

    return resp[1] == 1;
}

// ---------- Public API ----------

// Enable/disable driver (F3)
bool Servo42C::enable(bool on) {
    return writeCmd1Byte(0xF3, on ? 0x01 : 0x00);
}

// Constant speed run (F6)
bool Servo42C::runConstantSpeed(Direction dir, uint8_t speed) {
    speed &= 0x7F; // 7 bits
    uint8_t val = speed | (dir == CCW ? 0x80 : 0x00);

    uint8_t frame[4];
    frame[0] = _addr;
    frame[1] = 0xF6;
    frame[2] = val;
    frame[3] = checksum8(frame, 3);

    if (!writeCommand(frame, sizeof(frame))) return false;

    uint8_t resp[3];
    if (!readBytes(resp, sizeof(resp))) return false;

    if (resp[0] != _addr) return false;
    uint8_t crc = checksum8(resp, 2);
    if (crc != resp[2])   return false;

    return resp[1] == 1;
}

// Stop motor (F7)
bool Servo42C::stop() {
    return writeSimpleCmd(0xF7);
}

// Run by pulses (FD)
bool Servo42C::runSteps(Direction dir, uint8_t speed, uint32_t pulses) {
    speed &= 0x7F;
    uint8_t val = speed | (dir == CCW ? 0x80 : 0x00);

    uint8_t frame[8];
    frame[0] = _addr;
    frame[1] = 0xFD;
    frame[2] = val;
    // pulses big-endian
    frame[3] = (uint8_t)((pulses >> 24) & 0xFF);
    frame[4] = (uint8_t)((pulses >> 16) & 0xFF);
    frame[5] = (uint8_t)((pulses >>  8) & 0xFF);
    frame[6] = (uint8_t)((pulses >>  0) & 0xFF);
    frame[7] = checksum8(frame, 7);

    if (!writeCommand(frame, sizeof(frame))) return false;

    uint8_t resp[3];
    if (!readBytes(resp, sizeof(resp))) return false;

    if (resp[0] != _addr) return false;
    uint8_t crc = checksum8(resp, 2);
    if (crc != resp[2])   return false;

    // status: 0=fail, 1=running, 2=done. Treat 0 as error.
    return resp[1] != 0;
}

// Read shaft error (39)
bool Servo42C::readError(uint16_t& err) {
    uint8_t frame[3];
    frame[0] = _addr;
    frame[1] = 0x39;
    frame[2] = checksum8(frame, 2);

    if (!writeCommand(frame, sizeof(frame))) return false;

    uint8_t resp[4];
    if (!readBytes(resp, sizeof(resp))) return false;
    if (resp[0] != _addr) return false;
    uint8_t crc = checksum8(resp, 3);
    if (crc != resp[3])   return false;

    err = (uint16_t)((resp[1] << 8) | resp[2]);
    return true;
}
