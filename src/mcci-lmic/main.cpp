#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "secrets.hpp"
#include "utils.hpp"
#include "diagnostics.hpp"

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
uint16_t tx_success_count = 0;
uint16_t tx_fail_count = 0;
volatile bool dio0_rising = false;
void IRAM_ATTR dio0_isr() { dio0_rising = true; }
void do_send(osjob_t* j);
static const SPISettings sx1276SPI(8000000, MSBFIRST, SPI_MODE0);

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
    LMIC_setDrTxpow(DR_SF12, 14);
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

