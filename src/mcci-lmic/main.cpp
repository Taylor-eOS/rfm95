#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "secrets.hpp"

#define PIN_CS 5
#define PIN_RST 14
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK 18
#define PIN_DIO0 26
#define REG_VERSION 0x42
#define REG_OP_MODE 0x01
#define REG_IRQ_FLAGS 0x12
#define REG_IRQ_FLAGS_MASK 0x11
#define RF_OPMODE_STANDBY 0x01
#define RF_OPMODE_LORA_MODE 0x80
#define IRQ_TX_DONE_MASK 0x08
#define BAND_CENTI 0
#define DR_SF7    5
#define DR_SF8    4
#define DR_SF9    3
#define DR_SF10   2
#define DR_SF11   1
#define DR_SF12   0
#define LED_PIN 2

const lmic_pinmap lmic_pins = {
    .nss = PIN_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PIN_RST,
    .dio = {PIN_DIO0, 33, 32},
};

static const SPISettings sx1276SPI(8000000, MSBFIRST, SPI_MODE0);
osjob_t sendjob;
volatile bool dio0_rising = false;
void IRAM_ATTR dio0_isr() { dio0_rising = true; }
static uint8_t ping_payload[32] = "TTN_PING";
static uint16_t ping_counter = 0;
static bool transmission_pending = false;
static unsigned long last_tx_time = 0;
const unsigned long TX_INTERVAL = 20000UL;
const unsigned long LED_BUSY = 100UL;
const unsigned long LED_IDLE = 200UL;
static unsigned long led_last_toggle = 0;
static bool led_state = LOW;

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
    delay(20);
    digitalWrite(PIN_RST, HIGH);
    delay(100);
    pinMode(PIN_RST, INPUT);
}

void radio_preinit() {
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    delay(10);
    resetChip();
    uint8_t version = 0;
    for (int i = 0; i < 5; ++i) {
        version = readRegister(REG_VERSION);
        char b[64];
        int n = snprintf(b, sizeof(b), "SX127x VERSION: 0x%02X\n", version);
        if (n > 0) Serial.write((const uint8_t*)b, n);
        if (version == 0x12) break;
        resetChip();
        delay(200);
    }
    writeRegister(REG_OP_MODE, (uint8_t)(RF_OPMODE_LORA_MODE | RF_OPMODE_STANDBY));
    delay(10);
    uint8_t op = readRegister(REG_OP_MODE);
    {
        char b[48];
        int n = snprintf(b, sizeof(b), "OPMODE: 0x%02X\n", op);
        if (n > 0) Serial.write((const uint8_t*)b, n);
    }
    writeRegister(REG_IRQ_FLAGS, 0xFF); //clear
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t)(~IRQ_TX_DONE_MASK)); //unmask TXDONE
    pinMode(PIN_DIO0, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_DIO0), dio0_isr, RISING);
    delay(20);
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

void ensure_abp_session_and_defaults() {
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
    LMIC_setLinkCheckMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    LMIC_setAdrMode(0);
    LMIC_setDrTxpow(DR_SF12, 14);
    LMIC.txpow = 14;
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        char b[64];
        int n = snprintf(b, sizeof(b), "OP_TXRXPEND, skipping send\n");
        if (n > 0) Serial.write((const uint8_t*)b, n);
        return;
    }
    ping_counter++;
    snprintf((char*)ping_payload, sizeof(ping_payload), "PING_%04u", ping_counter);
    char line[80];
    int len = snprintf(line, sizeof(line), "SEND id:%u len:%u\n", ping_counter, (unsigned)strlen((char*)ping_payload));
    if (len > 0) Serial.write((const uint8_t*)line, len);
    LMIC_setTxData2(1, ping_payload, strlen((char*)ping_payload), 0);
    transmission_pending = true;
    last_tx_time = millis();
}

void onEvent(ev_t ev) {
    switch (ev) {
        case EV_TXSTART: {
            char s[48];
            int n = snprintf(s, sizeof(s), "EV_TXSTART id:%u\n", ping_counter);
            if (n > 0) Serial.write((const uint8_t*)s, n);
            break;
        }
        case EV_TXCOMPLETE: {
            transmission_pending = false;
            bool gotAck = (LMIC.txrxFlags & TXRX_ACK);
            bool hwEdge = false;
            noInterrupts();
            hwEdge = dio0_rising;
            dio0_rising = false;
            interrupts();
            char buf[200];
            int n = snprintf(buf, sizeof(buf),
                "EV_TXCOMPLETE id:%u %s DIO0_ISR:%s RSSI:%d SNR:%.1f\n",
                ping_counter,
                gotAck ? "ACK" : "NOACK",
                hwEdge ? "YES" : "NO",
                LMIC.rssi,
                LMIC.snr);
            if (n > 0) Serial.write((const uint8_t*)buf, n);
            //no reschedule; one-shot send
            break;
        }
        default: {
            char e[48];
            int n = snprintf(e, sizeof(e), "EV:%d\n", (int)ev);
            if (n > 0) Serial.write((const uint8_t*)e, n);
            break;
        }
    }
}

void led_init() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    led_last_toggle = millis();
    led_state = LOW;
}

void led_task() {
    unsigned long now = millis();
    bool busy = transmission_pending || (LMIC.opmode & OP_TXRXPEND);
    unsigned long on = busy ? LED_BUSY : LED_IDLE;
    unsigned long off = busy ? LED_BUSY : LED_IDLE * 10;
    unsigned long dur = led_state ? on : off;
    if (now - led_last_toggle >= dur) {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state ? HIGH : LOW);
        led_last_toggle = now;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) yield();
    delay(200);
    led_init();
    Serial.write((const uint8_t*)"Boot: radio preinit\n", 20);
    radio_preinit();
    os_init();
    LMIC_reset();
    ensure_abp_session_and_defaults();
    setup_eu868_channels();
    char s[80];
    int n = snprintf(s, sizeof(s), "LMIC configured ABP DEVADDR=0x%08lX SF12 P14\n", (unsigned long)DEVADDR);
    if (n > 0) Serial.write((const uint8_t*)s, n);
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
}

void loop() {
    led_task();
    os_runloop_once();
    if (transmission_pending && (millis() - last_tx_time > (TX_INTERVAL * 3))) {
        char t[80];
        int n = snprintf(t, sizeof(t), "TX timeout, giving up for this boot\n");
        if (n > 0) Serial.write((const uint8_t*)t, n);
        transmission_pending = false;
    }
}

