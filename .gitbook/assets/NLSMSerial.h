#pragma once

#include "Arduino.h"

namespace rath {

  #define NLSM_END              0x0AU
  #define NLSM_ESC              0x1BU
  #define NLSM_ESC_END          0x1CU
  #define NLSM_ESC_ESC          0x1DU

  class NLSMSerialClass {
    public:
      void init(long baudrate) {
        Serial.begin(baudrate);
      }
      
      uint16_t receive(uint8_t *buffer, uint16_t size) {
        uint8_t c;
        uint16_t index = 0;

        while (!Serial.available()) {}
        c = Serial.read();
        while (c != NLSM_END) {
          if (c == NLSM_ESC) {
            while (!Serial.available()) {}
            c = Serial.read();
            if (c == NLSM_ESC_END) {
              buffer[index] = NLSM_END;
            }
            else if (c == NLSM_ESC_ESC) {
              buffer[index] = NLSM_ESC;
            }
            else {
              buffer[index] = c;
            }
          }
          else {
            buffer[index] = c;
          }
          index += 1;
          while (!Serial.available()) {}
          c = Serial.read();
        }
        return index;
      }
      
      void transmit(uint8_t *buffer, uint16_t size) {
        if (size == 0) return ;
        uint8_t c;
        uint16_t index = 0;
        while (index < size) {
          c = buffer[index];
          if (c == NLSM_END) {
            c = NLSM_ESC;
            Serial.write(c);
            c = NLSM_ESC_END;
            Serial.write(c);
          }
          else if (c == NLSM_ESC) {
            c = NLSM_ESC;
            Serial.write(c);
            c = NLSM_ESC_ESC;
            Serial.write(c);
          }
          else {
            Serial.write(c);
          }
          index += 1;
        }
        c = NLSM_END;
        Serial.write(c);
      }
  };
}

rath::NLSMSerialClass NLSMSerial;
