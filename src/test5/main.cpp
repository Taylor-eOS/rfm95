#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "secrets.hpp"

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

osjob_t sendjob;

struct TestConfig {
    dr_t datarate;
    int8_t txpower;
    uint8_t payload_size;
    const char* description;
};

TestConfig test_configs[] = {
    {DR_SF10, 2, 4, "SF10, 2dBm, 4 bytes (least demanding)"},
    {DR_SF10, 14, 4, "SF10, 14dBm, 4 bytes"},
    {DR_SF9, 2, 4, "SF9, 2dBm, 4 bytes"},
    {DR_SF9, 14, 4, "SF9, 14dBm, 4 bytes"},
    {DR_SF8, 2, 4, "SF8, 2dBm, 4 bytes"},
    {DR_SF8, 14, 4, "SF8, 14dBm, 4 bytes"},
    {DR_SF7, 2, 4, "SF7, 2dBm, 4 bytes"},
    {DR_SF7, 14, 4, "SF7, 14dBm, 4 bytes"},
    {DR_SF7, 14, 10, "SF7, 14dBm, 10 bytes"},
    {DR_SF7, 14, 20, "SF7, 14dBm, 20 bytes"},
    {DR_SF7, 14, 50, "SF7, 14dBm, 50 bytes"},
    {DR_SF7, 14, 100, "SF7, 14dBm, 100 bytes (most demanding)"}
};

const int num_tests = sizeof(test_configs) / sizeof(test_configs[0]);
int current_test = 0;
uint8_t* test_payload = nullptr;
bool test_in_progress = false;
bool waiting_for_result = false;
unsigned long test_start_time = 0;
const unsigned long TEST_TIMEOUT = 30000;

struct TestResult {
    bool tx_started;
    bool tx_completed;
    bool timed_out;
    unsigned long duration_ms;
    int rssi;
    int snr;
};

TestResult results[12];

void print_summary();
void do_send(osjob_t* j);
void start_next_test();

void generate_test_payload(uint8_t size) {
    if (test_payload) {
        free(test_payload);
    }
    test_payload = (uint8_t*)malloc(size);
    for (uint8_t i = 0; i < size; i++) {
        test_payload[i] = (i % 26) + 'A';
    }
    if (size >= 4) {
        test_payload[0] = 'T';
        test_payload[1] = 'S';
        test_payload[2] = 'T';
        test_payload[3] = '0' + (current_test % 10);
    }
}

void next_test_callback(osjob_t* j) {
    start_next_test();
}

void start_next_test() {
    if (current_test >= num_tests) {
        Serial.println("\n=== ALL TESTS COMPLETED ===");
        print_summary();
        return;
    }
    TestConfig& config = test_configs[current_test];
    Serial.println();
    Serial.print("=== TEST ");
    Serial.print(current_test + 1);
    Serial.print("/");
    Serial.print(num_tests);
    Serial.print(": ");
    Serial.print(config.description);
    Serial.println(" ===");
    results[current_test] = {false, false, false, 0, 0, 0};
    generate_test_payload(config.payload_size);
    LMIC_setDrTxpow(config.datarate, config.txpower);
    Serial.print("Payload: ");
    for (int i = 0; i < config.payload_size; i++) {
        if (test_payload[i] >= 32 && test_payload[i] <= 126) {
            Serial.print((char)test_payload[i]);
        } else {
            Serial.print(".");
        }
    }
    Serial.print(" (");
    Serial.print(config.payload_size);
    Serial.println(" bytes)");
    test_in_progress = true;
    waiting_for_result = false;
    test_start_time = millis();
    do_send(&sendjob);
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("Previous operation still pending, retrying in 1s...");
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
        return;
    }
    if (!test_in_progress) {
        return;
    }
    TestConfig& config = test_configs[current_test];
    LMIC_setTxData2(1, test_payload, config.payload_size, 0);
    Serial.println("Packet queued for transmission");
    waiting_for_result = true;
}

void onEvent(ev_t ev) {
    unsigned long current_time = millis();
    switch (ev) {
        case EV_TXSTART:
            Serial.print("[");
            Serial.print(current_time);
            Serial.println("ms] TX START");
            if (test_in_progress) {
                results[current_test].tx_started = true;
            }
            break;
        case EV_TXCOMPLETE:
            Serial.print("[");
            Serial.print(current_time);
            Serial.print("ms] TX COMPLETE");
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.print(" (ACK received)");
            }
            if (LMIC.dataLen) {
                Serial.print(" (");
                Serial.print(LMIC.dataLen);
                Serial.print(" bytes received)");
            }
            Serial.println();
            if (test_in_progress) {
                results[current_test].tx_completed = true;
                results[current_test].duration_ms = current_time - test_start_time;
                results[current_test].rssi = LMIC.rssi;
                results[current_test].snr = LMIC.snr;
                Serial.print("RSSI: ");
                Serial.print(LMIC.rssi);
                Serial.print(" dBm, SNR: ");
                Serial.print(LMIC.snr);
                Serial.println(" dB");
                Serial.println("TEST PASSED");
                test_in_progress = false;
                current_test++;
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), next_test_callback);
            }
            break;
        case EV_TXCANCELED:
            Serial.print("[");
            Serial.print(current_time);
            Serial.println("ms] TX CANCELED");
            if (test_in_progress) {
                Serial.println("TEST FAILED (CANCELED)");
                test_in_progress = false;
                current_test++;
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), next_test_callback);
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println("Lost time sync");
            break;
        case EV_RESET:
            Serial.println("Radio reset");
            break;
        case EV_RXCOMPLETE:
            Serial.println("RX complete");
            break;
        case EV_LINK_DEAD:
            Serial.println("Link dead");
            break;
        case EV_LINK_ALIVE:
            Serial.println("Link alive");
            break;
        case EV_SCAN_FOUND:
            Serial.println("Scan found");
            break;
        case EV_BEACON_FOUND:
            Serial.println("Beacon found");
            break;
        case EV_BEACON_MISSED:
            Serial.println("Beacon missed");
            break;
        case EV_BEACON_TRACKED:
            Serial.println("Beacon tracked");
            break;
        case EV_RFU1:
            Serial.println("RFU1");
            break;
        case EV_JOIN_FAILED:
            Serial.println("Join failed");
            break;
        case EV_REJOIN_FAILED:
            Serial.println("Rejoin failed");
            break;
        default:
            Serial.print("Unknown event: ");
            Serial.println((unsigned) ev);
            break;
    }
}

void print_summary() {
    Serial.println("\n=== TEST SUMMARY ===");
    Serial.println("Config\t\t\t\tStarted\tCompleted\tDuration\tRSSI\tSNR");
    Serial.println("--------------------------------------------------------------------------------");
    for (int i = 0; i < num_tests; i++) {
        Serial.print(test_configs[i].description);
        int len = strlen(test_configs[i].description);
        for (int j = len; j < 40; j++) Serial.print(" ");
        Serial.print("\t");
        Serial.print(results[i].tx_started ? "YES" : "NO");
        Serial.print("\t");
        Serial.print(results[i].tx_completed ? "YES" : "NO");
        Serial.print("\t\t");
        if (results[i].tx_completed) {
            Serial.print(results[i].duration_ms);
            Serial.print("ms\t\t");
            Serial.print(results[i].rssi);
            Serial.print("\t");
            Serial.print(results[i].snr);
        } else {
            Serial.print("N/A\t\t-\t-");
        }
        Serial.println();
    }
    Serial.println("\n=== ANALYSIS ===");
    int successful_tests = 0;
    int last_successful = -1;
    for (int i = 0; i < num_tests; i++) {
        if (results[i].tx_completed) {
            successful_tests++;
            last_successful = i;
        }
    }
    Serial.print("Successful transmissions: ");
    Serial.print(successful_tests);
    Serial.print("/");
    Serial.println(num_tests);
    if (last_successful >= 0) {
        Serial.print("Most demanding successful config: ");
        Serial.println(test_configs[last_successful].description);
    }
    if (successful_tests == 0) {
        Serial.println("No transmissions succeeded - check hardware, keys, or coverage");
    } else if (successful_tests < num_tests) {
        Serial.println("Some transmissions failed - there's a limit to transmission parameters");
    } else {
        Serial.println("All transmissions succeeded - your setup can handle demanding configs");
    }
}

void check_test_timeout() {
    if (test_in_progress && waiting_for_result) {
        if (millis() - test_start_time > TEST_TIMEOUT) {
            Serial.println("Test timeout");
            results[current_test].timed_out = true;
            test_in_progress = false;
            current_test++;
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), next_test_callback);
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) yield();
    delay(200);
    Serial.println("=== LoRaWAN Diagnostic Tool ===");
    Serial.println("This tool will test various transmission parameters");
    Serial.println("to identify the limits of your LoRaWAN setup.\n");
    SPI.begin();
    os_init();
    LMIC_reset();
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
    for (int i = 0; i <= 15; ++i) LMIC_disableChannel(i);
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF10, DR_SF7), 0);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF10, DR_SF7), 0);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF10, DR_SF7), 0);
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    Serial.println("Starting diagnostic tests in 5 seconds...");
    delay(5000);
    start_next_test();
}

void loop() {
    os_runloop_once();
    check_test_timeout();
}

