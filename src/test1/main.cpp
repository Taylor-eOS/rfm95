#include <Arduino.h>
#include <SPI.h>

#define PIN_CS    5
#define PIN_RST   14
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK  18

uint8_t readRegister(uint8_t addr) {
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(addr & 0x7F);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(PIN_CS, HIGH);
    return val;
}

void resetChip() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(10);
    digitalWrite(PIN_RST, HIGH);
    delay(10);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    resetChip();
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    delay(100);

    uint8_t version = readRegister(0x42);
    Serial.print("SX127x Version register: 0x");
    Serial.println(version, HEX);
    if (version == 0x12) {
        Serial.println("Chip detected OK");
    } else {
        Serial.println("Unexpected response, check wiring or power");
    }
}

void loop() {
}

