#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "secrets.hpp"

#define LMIC_USE_INTERRUPTS
#define LMIC_DEBUG_LEVEL 2
#define LMIC_PRINTF_TO Serial

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
    .spi_freq = 8000000,
};
osjob_t sendjob;
const uint32_t CHANNELS[] = {868100000, 868300000, 868500000};
const uint8_t NUM_CHANNELS = 3;
const uint8_t DEFAULT_DR = DR_SF7;
const uint8_t DEFAULT_POWER = 14;
const unsigned long TX_INTERVAL = 10000;
uint16_t ping_counter = 0;
unsigned long last_tx_time = 0;
bool transmission_pending = false;

void setup_eu868_channels() {
    LMIC_reset();
    for (int i = 0; i < 72; ++i) LMIC_disableChannel(i);
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        LMIC_setupChannel(i, CHANNELS[i], DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
        LMIC_enableChannel(i);
    }
    //Set initial data rate and power
    LMIC_setDrTxpow(DEFAULT_DR, DEFAULT_POWER);
    //ABP session
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
    //Disable link check and ADR (enable ADR if needed)
    LMIC_setLinkCheckMode(0);
    LMIC_setAdrMode(0);
    //Clock error tolerance for ESP32
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    //Set RX2 to SF9 (TTN default)
    LMIC.dn2Dr = DR_SF9;
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("TX busy, retrying in 2s"));
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
        return;
    }
    ping_counter++;
    char payload[32];
    snprintf(payload, sizeof(payload), "PING%04d_SF%d_P%d", ping_counter, 12 - (LMIC.datarate - DR_SF7), DEFAULT_POWER);
    Serial.printf("Queueing packet on CH%d (%.1f MHz): %s\n", LMIC.txChnl, LMIC.freq / 1e6, payload);
    LMIC_setTxData2(1, (uint8_t*)payload, strlen(payload), 0);
    transmission_pending = true;
    last_tx_time = millis();
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
        case EV_TXSTART:
            Serial.printf("EV_TXSTART - Transmitting on CH%d (%.1f MHz)\n", LMIC.txChnl, LMIC.freq / 1e6);
            break;
        case EV_TXCOMPLETE:
            transmission_pending = false;
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("EV_TXCOMPLETE with ACK"));
            } else {
                Serial.println(F("EV_TXCOMPLETE (includes RX windows)"));
            }
            if (LMIC.dataLen) {
                Serial.printf("Received %d bytes of payload\n", LMIC.dataLen);
            }
            //Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL / 1000), do_send);
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE"));
            break;
        default:
            Serial.printf("Unknown event: %d\n", (unsigned)ev);
            break;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);
    Serial.println(F("Starting LMIC test transmitter"));
    SPI.begin();
    os_init();
    setup_eu868_channels();
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
    //Timeout safety net for SX1276 clone quirks
    if (transmission_pending && (millis() - last_tx_time > 30000)) {
        Serial.println(F("TX timeout detected - resetting LMIC"));
        transmission_pending = false;
        setup_eu868_channels();
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
    }
}

