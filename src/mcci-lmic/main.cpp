#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "secrets.hpp"
#include <stdarg.h>

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

osjob_t sendjob;
volatile bool dio0_rising = false;
uint16_t ping_counter = 0;
bool transmission_pending = false;
unsigned long last_tx_time = 0;
const unsigned long TX_INTERVAL = 3000;
uint16_t tx_success_count = 0;
uint16_t tx_fail_count = 0;
uint16_t tx_timeout_count = 0;
void IRAM_ATTR dio0_isr() { dio0_rising = true; }

struct TestConfig {
    uint8_t channel;
    uint8_t dr;
    uint8_t power;
    uint32_t frequency;
};

#define NUM_CHANNELS 8
#define NUM_DR 6
#define NUM_POWER 3
TestConfig testConfigs[NUM_CHANNELS * NUM_DR * NUM_POWER];
uint16_t totalTests = 0;
uint16_t currentTest = 0;
unsigned long last_status_time = 0;
const unsigned long STATUS_INTERVAL = 60000;

void serPrintln(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.print(buf);
    Serial.print("\r\n");
    Serial.flush();
}

void prepareTestConfigs() {
    totalTests = 0;
    uint32_t frequencies[8] = {
        868100000, 868300000, 868500000, 867100000,
        867300000, 867500000, 867700000, 867900000
    };
    uint8_t powers[3] = {2, 8, 14};
    uint8_t datarates[6] = {DR_SF12, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7};
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        for (uint8_t dr_idx = 0; dr_idx < NUM_DR; dr_idx++) {
            for (uint8_t p = 0; p < NUM_POWER; p++) {
                testConfigs[totalTests].channel = ch;
                testConfigs[totalTests].dr = datarates[dr_idx];
                testConfigs[totalTests].power = powers[p];
                testConfigs[totalTests].frequency = frequencies[ch];
                totalTests++;
            }
        }
    }
    serPrintln("Generated %d test configurations", totalTests);
}

void setup_eu868_channels() {
    for (int i = 0; i < 72; ++i) LMIC_disableChannel(i);
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        tx_fail_count++;
        serPrintln("TX busy");
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
        return;
    }
    TestConfig cfg = testConfigs[currentTest];
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (i == cfg.channel) LMIC_enableChannel(i);
        else LMIC_disableChannel(i);
    }
    LMIC_setDrTxpow(cfg.dr, cfg.power);
    ping_counter++;
    char payload[64];
    snprintf(payload, sizeof(payload), "TEST%04d_CH%d_SF%d_PWR%d_F%lu",
        ping_counter,
        cfg.channel,
        12 - (cfg.dr - DR_SF7),
        cfg.power,
        cfg.frequency / 1000000);
    serPrintln("Sending test %d/%d: %s", currentTest + 1, totalTests, payload);
    LMIC_setTxData2(1, (uint8_t*)payload, strlen(payload), 0);
    transmission_pending = true;
    last_tx_time = millis();
    currentTest = (currentTest + 1) % totalTests;
}

void onEvent(ev_t ev) {
    switch (ev) {
        case EV_TXCOMPLETE:
            transmission_pending = false;
            if (LMIC.txrxFlags & TXRX_ACK) {
                tx_success_count++;
                serPrintln("TX complete with ACK (success: %d)", tx_success_count);
            } else {
                serPrintln("TX complete no ACK (sent: %d)", ping_counter);
            }
            os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(TX_INTERVAL), do_send);
            break;
        case EV_TXSTART:
            serPrintln("TX started");
            break;
        case EV_JOIN_TXCOMPLETE:
            serPrintln("Join TX complete");
            break;
        default:
            serPrintln("Event: %d", (int)ev);
            break;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) yield();
    delay(1000);
    pinMode(lmic_pins.dio[0], INPUT);
    pinMode(lmic_pins.dio[1], INPUT);
    pinMode(lmic_pins.dio[2], INPUT);
    SPI.begin();
    attachInterrupt(digitalPinToInterrupt(lmic_pins.dio[0]), dio0_isr, RISING);
    os_init();
    LMIC_reset();
    LMIC_setLinkCheckMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    LMIC_setAdrMode(0);
    prepareTestConfigs();
    setup_eu868_channels();
    serPrintln("Starting test cycle with %d configurations", totalTests);
    last_status_time = millis();
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), do_send);
}

void loop() {
    os_runloop_once();
    if (millis() - last_tx_time > 30000 && transmission_pending) {
        serPrintln("TX timeout - resetting (timeouts: %d)", ++tx_timeout_count);
        transmission_pending = false;
        LMIC_reset();
        setup_eu868_channels();
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
    }
    if (millis() - last_status_time >= STATUS_INTERVAL) {
        last_status_time = millis();
        serPrintln("Status - Sent: %d, ACKs: %d, Fails: %d, Timeouts: %d", 
            ping_counter, tx_success_count, tx_fail_count, tx_timeout_count);
    }
}

