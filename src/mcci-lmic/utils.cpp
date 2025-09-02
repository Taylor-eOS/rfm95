#include "utils.hpp"
#include <Arduino.h>

void print_hex(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (data[i] < 16) Serial.print("0");
        Serial.print(data[i], HEX);
        if (i < len - 1) Serial.print(" ");
    }
}
