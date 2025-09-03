#include "diagnostics.hpp"

static const SPISettings sx1276SPI(8000000, MSBFIRST, SPI_MODE0);

uint8_t readRegister(uint8_t address) {
    digitalWrite(PIN_CS, LOW);
    SPI.beginTransaction(sx1276SPI);
    SPI.transfer(address & 0x7F);
    uint8_t val = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(PIN_CS, HIGH);
    return val;
}

void writeRegister(uint8_t address, uint8_t value) {
    digitalWrite(PIN_CS, LOW);
    SPI.beginTransaction(sx1276SPI);
    SPI.transfer(address | 0x80);
    SPI.transfer(value);
    SPI.endTransaction();
    digitalWrite(PIN_CS, HIGH);
}

void resetChip() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(10);
    digitalWrite(PIN_RST, HIGH);
    delay(50);
    pinMode(PIN_RST, INPUT);
}

void setup_eu868_channels() {
    for (int i = 0; i < 72; ++i) {
        LMIC_disableChannel(i);
    }
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);
}

void print_device_info() {
    Serial.println("=== Device Configuration ===");
    Serial.println();
    Serial.println("Region: EU868");
    Serial.println("Activation: ABP");
    Serial.println();
}

void print_channel_status() {
    Serial.println("=== Active Channels ===");
    for (int i = 0; i < 9; i++) {
        if (LMIC.channelMap & (1 << i)) {
            Serial.print("Channel ");
            Serial.print(i);
            Serial.print(": ");
            switch(i) {
                case 0: Serial.println("868.1 MHz"); break;
                case 1: Serial.println("868.3 MHz"); break;
                case 2: Serial.println("868.5 MHz"); break;
                case 3: Serial.println("867.1 MHz"); break;
                case 4: Serial.println("867.3 MHz"); break;
                case 5: Serial.println("867.5 MHz"); break;
                case 6: Serial.println("867.7 MHz"); break;
                case 7: Serial.println("867.9 MHz"); break;
                case 8: Serial.println("868.8 MHz (FSK)"); break;
            }
        }
    }
    Serial.println();
}

void print_lmic_status() {
    Serial.println("=== LMIC Status ===");
    Serial.print("Frame Counter Up: ");
    Serial.println(LMIC.seqnoUp);
    Serial.print("Frame Counter Down: ");
    Serial.println(LMIC.seqnoDn);
    Serial.print("Current Datarate: SF");
    switch(LMIC.datarate) {
        case DR_SF7: Serial.println("7"); break;
        case DR_SF8: Serial.println("8"); break;
        case DR_SF9: Serial.println("9"); break;
        case DR_SF10: Serial.println("10"); break;
        case DR_SF11: Serial.println("11"); break;
        case DR_SF12: Serial.println("12"); break;
        default: Serial.println("Unknown"); break;
    }
    Serial.print("TX Power: ");
    Serial.print(LMIC.txpow);
    Serial.println(" dBm");
    Serial.print("Adaptive Data Rate: ");
    Serial.println(LMIC.adrEnabled ? "Enabled" : "Disabled");
    Serial.print("Duty Cycle: ");
    Serial.print(LMIC.globalDutyRate);
    Serial.println("%");
    Serial.println();
}

void print_transmission_stats() {
    Serial.println("=== Transmission Statistics ===");
    Serial.print("Successful transmissions: ");
    Serial.println(tx_success_count);
    Serial.print("Failed transmissions: ");
    Serial.println(tx_fail_count);
    if (tx_success_count + tx_fail_count > 0) {
        Serial.print("Success rate: ");
        Serial.print((100.0 * tx_success_count) / (tx_success_count + tx_fail_count));
        Serial.println("%");
    }
    Serial.println();
}

