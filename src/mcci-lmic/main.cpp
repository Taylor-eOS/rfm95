#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "secrets.hpp"
#include "utils.hpp"

#define PIN_CS 5
#define PIN_RST 14
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK 18
#define PIN_DIO0 26
#define REG_VERSION 0x42
#define REG_OP_MODE 0x01
#define REG_FIFO 0x00
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_ADDR 0x0E
#define REG_PAYLOAD_LENGTH 0x22
#define REG_IRQ_FLAGS 0x12
#define REG_IRQ_FLAGS_MASK 0x11
#define RF_OPMODE_SLEEP 0x00
#define RF_OPMODE_STANDBY 0x01
#define RF_OPMODE_TX 0x03
#define RF_OPMODE_LORA_MODE 0x80
#define IRQ_TX_DONE_MASK 0x08

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};
osjob_t sendjob;
static uint8_t ping_payload[] = "TTN_PING";
static uint16_t ping_counter = 0;
bool transmission_pending = false;
unsigned long last_tx_time = 0;
const unsigned long TX_INTERVAL = 10000;
static uint16_t tx_success_count = 0;
static uint16_t tx_fail_count = 0;
volatile bool dio0_rising = false;
void IRAM_ATTR dio0_isr() { dio0_rising = true; }
void do_send(osjob_t* j);
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

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
        return;
    }
    ping_counter++;
    sprintf((char*)ping_payload, "PING_%04d", ping_counter);
    Serial.print("Sending: ");
    Serial.print((char*)ping_payload);
    Serial.print(" (");
    Serial.print(strlen((char*)ping_payload));
    Serial.println(" bytes)");
    Serial.print("Using DR: SF");
    switch(LMIC.datarate) {
        case DR_SF7: Serial.print("7"); break;
        case DR_SF8: Serial.print("8"); break;
        case DR_SF9: Serial.print("9"); break;
        case DR_SF10: Serial.print("10"); break;
        case DR_SF11: Serial.print("11"); break;
        case DR_SF12: Serial.print("12"); break;
        default: Serial.print("?"); break;
    }
    LMIC.txpow = 14;
    Serial.print(", Power: ");
    Serial.print(LMIC.txpow);
    Serial.print(" dBm, Channel: ");
    Serial.println(LMIC.txChnl);
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("Error: Radio is busy, skipping transmission");
        tx_fail_count++;
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), do_send);
        return;
    }
    LMIC_setTxData2(1, ping_payload, strlen((char*)ping_payload), 0);
    transmission_pending = true;
    last_tx_time = millis();
}

void onEvent(ev_t ev) {
    unsigned long current_time = millis();
    Serial.print("[");
    Serial.print(current_time);
    Serial.print("ms] ");
    switch(ev) {
        case EV_SCAN_TIMEOUT: Serial.println("EV_SCAN_TIMEOUT"); break;
        case EV_BEACON_FOUND: Serial.println("EV_BEACON_FOUND"); break;
        case EV_BEACON_MISSED: Serial.println("EV_BEACON_MISSED"); break;
        case EV_BEACON_TRACKED: Serial.println("EV_BEACON_TRACKED"); break;
        case EV_JOINING: Serial.println("EV_JOINING"); break;
        case EV_JOINED: Serial.println("EV_JOINED"); break;
        case EV_RFU1: Serial.println("EV_RFU1"); break;
        case EV_JOIN_FAILED: Serial.println("EV_JOIN_FAILED"); break;
        case EV_REJOIN_FAILED: Serial.println("EV_REJOIN_FAILED"); break;
        case EV_TXSTART: Serial.println("EV_TXSTART - Radio transmission started"); break;
        case EV_TXCOMPLETE: {
            Serial.print("EV_TXCOMPLETE");
            transmission_pending = false;
            bool hwEdge = false;
            noInterrupts();
            hwEdge = dio0_rising;
            dio0_rising = false;
            interrupts();
            Serial.print(" - DIO0 edge seen by ISR: ");
            Serial.print(hwEdge ? "YES" : "NO");
            Serial.print(" - DIO0 current level: ");
            Serial.print(digitalRead(lmic_pins.dio[0]) ? "HIGH" : "LOW");
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.print(" - ACK received from TTN!");
                tx_success_count++;
            } else {
                Serial.print(" - No ACK");
                tx_success_count++;
            }
            if (LMIC.dataLen) {
                Serial.print(" - Received ");
                Serial.print(LMIC.dataLen);
                Serial.print(" bytes: ");
                for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 16) Serial.print("0");
                    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                }
            }
            Serial.println();
            Serial.print("RSSI: ");
            Serial.print(LMIC.rssi);
            Serial.print(" dBm, SNR: ");
            Serial.print(LMIC.snr);
            Serial.println(" dB");
            Serial.println("Transmission attempt to TTN complete");
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL/1000), do_send);
            break;
        }
        case EV_LOST_TSYNC: Serial.println("EV_LOST_TSYNC"); break;
        case EV_RESET: Serial.println("EV_RESET"); break;
        case EV_RXCOMPLETE: Serial.println("EV_RXCOMPLETE"); break;
        case EV_LINK_DEAD: Serial.println("EV_LINK_DEAD"); break;
        case EV_LINK_ALIVE: Serial.println("EV_LINK_ALIVE"); break;
        case EV_SCAN_FOUND: Serial.println("EV_SCAN_FOUND"); break;
        case EV_TXCANCELED:
            Serial.println("EV_TXCANCELED");
            transmission_pending = false;
            tx_fail_count++;
            Serial.println("Transmission canceled");
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(10), do_send);
            break;
        default:
            Serial.print("Unknown event: ");
            Serial.println((unsigned) ev);
            break;
    }
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

void setup() {
    Serial.begin(115200);
    while (!Serial) yield();
    delay(1000);
    Serial.println("=== TTN Ping Test ===");
    Serial.println("Testing connection to TTN");
    Serial.println();
    print_device_info();
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(lmic_pins.dio[0], INPUT);
    pinMode(lmic_pins.dio[1], INPUT);
    pinMode(lmic_pins.dio[2], INPUT);
    resetChip();
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    delay(100);
    uint8_t version = 0;
    for (int i = 0; i < 5; ++i) {
        version = readRegister(REG_VERSION);
        Serial.print("SX127x Version register: 0x");
        Serial.println(version, HEX);
        if (version == 0x12) break;
        resetChip();
        delay(100);
    }
    if (version == 0x12) {
        Serial.println("Chip detected OK.");
    } else {
        Serial.println("Unexpected response from radio; continuing but check wiring and reset line.");
    }
    writeRegister(REG_OP_MODE, RF_OPMODE_LORA_MODE | RF_OPMODE_STANDBY);
    delay(10);
    Serial.print("Current Op Mode: 0x");
    Serial.println(readRegister(REG_OP_MODE), HEX);
    writeRegister(REG_IRQ_FLAGS_MASK, ~IRQ_TX_DONE_MASK);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    attachInterrupt(digitalPinToInterrupt(lmic_pins.dio[0]), dio0_isr, RISING);
    delay(20);
    os_init();
    LMIC_reset();
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
    setup_eu868_channels();
    print_channel_status();
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC.txpow = 14;
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    LMIC.adrTxPow = 14;
    LMIC_setAdrMode(0);
    print_lmic_status();
    Serial.println("Starting TTN ping test...");
    Serial.print("Will send ping every ");
    Serial.print(TX_INTERVAL / 1000);
    Serial.println(" seconds");
    Serial.println();
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
}

void loop() {
    os_runloop_once();
    static unsigned long last_status = 0;
    if (millis() - last_status > 30000) {
        last_status = millis();
        if (transmission_pending) {
            Serial.print("Transmission pending for ");
            Serial.print((millis() - last_tx_time) / 1000);
            Serial.println(" seconds...");
        } else {
            Serial.print("Next transmission in ");
            Serial.print((TX_INTERVAL - (millis() - last_tx_time)) / 1000);
            Serial.println(" seconds");
        }
        print_transmission_stats();
    }
}

