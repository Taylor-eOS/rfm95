#include <Arduino.h>
#include <SPI.h>
#include "secrets.hpp"

#define SX1276_NSS 5
#define SX1276_RST 14
#define SX1276_DIO0 26
#define LORAWAN_FPORT 1
#define TX_BW 7
#define TX_CR 1
#define TX_PREAMBLE 8
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FR_MSB 0x06
#define REG_FR_MID 0x07
#define REG_FR_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_FIFO_TX_BASE 0x0E
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_IRQ_FLAGS_MASK 0x11
#define REG_IRQ_FLAGS 0x12
#define REG_PAYLOAD_LEN 0x22
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING1 0x40
#define MODE_SLEEP 0x00
#define MODE_STANDBY 0x01
#define MODE_TX 0x03
#define MODE_LONG_RANGE 0x80

static const uint8_t sbox[256] = {0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, 0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};

static uint8_t gmul(uint8_t a, uint8_t b) {
    uint8_t p = 0;
    for (int i = 0; i < 8; i++) {
        if (b & 1) p ^= a;
        bool hi = a & 0x80;
        a <<= 1;
        if (hi) a ^= 0x1b;
        b >>= 1;
    }
    return p;
}

static void aes_encrypt(const uint8_t* key, const uint8_t* in, uint8_t* out) {
    uint8_t state[16], rk[176];
    memcpy(state, in, 16);
    memcpy(rk, key, 16);
    for (int i = 16; i < 176; i += 4) {
        uint8_t t[4];
        memcpy(t, rk + i - 4, 4);
        if ((i % 16) == 0) {
            const uint8_t rcon[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36};
            uint8_t tmp = t[0];
            t[0] = sbox[t[1]] ^ rcon[(i / 16) - 1];
            t[1] = sbox[t[2]];
            t[2] = sbox[t[3]];
            t[3] = sbox[tmp];
        }
        for (int j = 0; j < 4; j++) rk[i + j] = rk[i - 16 + j] ^ t[j];
    }
    for (int j = 0; j < 16; j++) state[j] ^= rk[j];
    for (int r = 1; r <= 10; r++) {
        for (int j = 0; j < 16; j++) state[j] = sbox[state[j]];
        uint8_t tmp;
        tmp = state[1];
        state[1] = state[5];
        state[5] = state[9];
        state[9] = state[13];
        state[13] = tmp;
        tmp = state[2];
        state[2] = state[10];
        state[10] = tmp;
        tmp = state[6];
        state[6] = state[14];
        state[14] = tmp;
        tmp = state[15];
        state[15] = state[11];
        state[11] = state[7];
        state[7] = state[3];
        state[3] = tmp;
        if (r < 10) {
            for (int c = 0; c < 4; c++) {
                uint8_t* s = state + 4 * c;
                uint8_t a0 = s[0], a1 = s[1], a2 = s[2], a3 = s[3];
                s[0] = gmul(2, a0) ^ gmul(3, a1) ^ a2 ^ a3;
                s[1] = a0 ^ gmul(2, a1) ^ gmul(3, a2) ^ a3;
                s[2] = a0 ^ a1 ^ gmul(2, a2) ^ gmul(3, a3);
                s[3] = gmul(3, a0) ^ a1 ^ a2 ^ gmul(2, a3);
            }
        }
        for (int j = 0; j < 16; j++) state[j] ^= rk[r * 16 + j];
    }
    memcpy(out, state, 16);
}

static void lorawan_encrypt_payload(const uint8_t* appskey, uint32_t devaddr, uint8_t dir, uint32_t fcnt, const uint8_t* in, uint8_t len, uint8_t* out) {
    uint8_t block[16], s[16];
    for (uint8_t i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            uint8_t k = i / 16 + 1;
            block[0] = 0x01;
            block[1] = 0;
            block[2] = 0;
            block[3] = 0;
            block[4] = 0;
            block[5] = dir;
            block[6] = (devaddr) & 0xFF;
            block[7] = (devaddr >> 8) & 0xFF;
            block[8] = (devaddr >> 16) & 0xFF;
            block[9] = (devaddr >> 24) & 0xFF;
            block[10] = (fcnt) & 0xFF;
            block[11] = (fcnt >> 8) & 0xFF;
            block[12] = (fcnt >> 16) & 0xFF;
            block[13] = (fcnt >> 24) & 0xFF;
            block[14] = 0;
            block[15] = k;
            aes_encrypt(appskey, block, s);
        }
        out[i] = in[i] ^ s[i % 16];
    }
}

static void aes_cmac_generate_subkeys(const uint8_t* key, uint8_t* k1, uint8_t* k2) {
    const uint8_t zero[16] = {0};
    uint8_t L[16];
    aes_encrypt(key, zero, L);
    bool msb = L[0] & 0x80;
    for (int i = 0; i < 15; i++) k1[i] = (L[i] << 1) | (L[i + 1] >> 7);
    k1[15] = L[15] << 1;
    if (msb) k1[15] ^= 0x87;
    msb = k1[0] & 0x80;
    for (int i = 0; i < 15; i++) k2[i] = (k1[i] << 1) | (k1[i + 1] >> 7);
    k2[15] = k1[15] << 1;
    if (msb) k2[15] ^= 0x87;
}

static void lorawan_compute_mic(const uint8_t* nwkskey, uint32_t devaddr, uint32_t fcnt, const uint8_t* pkt, uint8_t pktlen, uint8_t mic[4]) {
    uint8_t b0[16] = {0};
    b0[0] = 0x49;
    b0[5] = 0x00;
    b0[6] = (devaddr) & 0xFF;
    b0[7] = (devaddr >> 8) & 0xFF;
    b0[8] = (devaddr >> 16) & 0xFF;
    b0[9] = (devaddr >> 24) & 0xFF;
    b0[10] = (fcnt) & 0xFF;
    b0[11] = (fcnt >> 8) & 0xFF;
    b0[12] = 0;
    b0[13] = 0;
    b0[14] = 0;
    b0[15] = pktlen;
    uint8_t k1[16], k2[16];
    aes_cmac_generate_subkeys(nwkskey, k1, k2);
    uint8_t x[16] = {0};
    for (int i = 0; i < 16; i++) x[i] ^= b0[i];
    aes_encrypt(nwkskey, x, x);
    int n_blocks = (pktlen + 15) / 16;
    bool last_complete = (pktlen % 16) == 0;
    for (int blk = 0; blk < n_blocks; blk++) {
        uint8_t m[16] = {0};
        int offset = blk * 16;
        int take = (pktlen - offset < 16) ? (pktlen - offset) : 16;
        memcpy(m, pkt + offset, take);
        bool is_last = (blk == n_blocks - 1);
        if (is_last) {
            if (!last_complete) m[take] = 0x80;
            const uint8_t* subkey = (is_last && last_complete) ? k1 : k2;
            for (int i = 0; i < 16; i++) m[i] ^= subkey[i];
        }
        for (int i = 0; i < 16; i++) x[i] ^= m[i];
        aes_encrypt(nwkskey, x, x);
    }
    memcpy(mic, x, 4);
}

static void sx_write(uint8_t reg, uint8_t val) {
    digitalWrite(SX1276_NSS, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(val);
    digitalWrite(SX1276_NSS, HIGH);
}

static uint8_t sx_read(uint8_t reg) {
    digitalWrite(SX1276_NSS, LOW);
    SPI.transfer(reg & 0x7F);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(SX1276_NSS, HIGH);
    return v;
}

static void sx_write_fifo(const uint8_t* buf, uint8_t len) {
    digitalWrite(SX1276_NSS, LOW);
    SPI.transfer(REG_FIFO | 0x80);
    for (int i = 0; i < len; i++) SPI.transfer(buf[i]);
    digitalWrite(SX1276_NSS, HIGH);
}

static void sx1276_init() {
    pinMode(SX1276_NSS, OUTPUT);
    pinMode(SX1276_RST, OUTPUT);
    pinMode(SX1276_DIO0, INPUT);
    digitalWrite(SX1276_NSS, HIGH);
    digitalWrite(SX1276_RST, LOW);
    delay(10);
    digitalWrite(SX1276_RST, HIGH);
    delay(10);
    sx_write(REG_OP_MODE, MODE_SLEEP | MODE_LONG_RANGE);
    delay(10);
    sx_write(REG_OP_MODE, MODE_STANDBY | MODE_LONG_RANGE);
    delay(10);
    sx_write(REG_PA_CONFIG, 0x8C);
    sx_write(REG_MODEM_CONFIG1, (TX_BW << 4) | (TX_CR << 1) | 0);
    sx_write(REG_PREAMBLE_MSB, 0x00);
    sx_write(REG_PREAMBLE_LSB, TX_PREAMBLE);
    sx_write(REG_SYNC_WORD, 0x34);
    sx_write(REG_DIO_MAPPING1, 0x40);
    sx_write(REG_FIFO_TX_BASE, 0x00);
    Serial.printf("SX1276 version: 0x%02X (expect 0x12)\n", sx_read(0x42));
}

static void apply_tx_params(uint32_t freq, uint8_t sf) {
    sx_write(REG_OP_MODE, MODE_STANDBY | MODE_LONG_RANGE);
    uint64_t frf = ((uint64_t)freq << 19) / 32000000UL;
    sx_write(REG_FR_MSB, (frf >> 16) & 0xFF);
    sx_write(REG_FR_MID, (frf >> 8) & 0xFF);
    sx_write(REG_FR_LSB, frf & 0xFF);
    sx_write(REG_MODEM_CONFIG2, (sf << 4) | 0x04);
}

static uint32_t fcnt = 0;
static void lorawan_send(const uint8_t* payload, uint8_t paylen, uint32_t freq, uint8_t sf) {
    uint8_t enc[64];
    lorawan_encrypt_payload(APPSKEY, DEVADDR, 0, fcnt, payload, paylen, enc);
    uint8_t frame[64];
    uint8_t idx = 0;
    frame[idx++] = 0x40;
    frame[idx++] = (DEVADDR) & 0xFF;
    frame[idx++] = (DEVADDR >> 8) & 0xFF;
    frame[idx++] = (DEVADDR >> 16) & 0xFF;
    frame[idx++] = (DEVADDR >> 24) & 0xFF;
    frame[idx++] = 0x00;
    frame[idx++] = fcnt & 0xFF;
    frame[idx++] = (fcnt >> 8) & 0xFF;
    frame[idx++] = LORAWAN_FPORT;
    memcpy(frame + idx, enc, paylen);
    idx += paylen;
    uint8_t mic[4];
    lorawan_compute_mic(NWKSKEY, DEVADDR, fcnt, frame, idx, mic);
    memcpy(frame + idx, mic, 4);
    idx += 4;
    fcnt++;
    sx_write(REG_OP_MODE, MODE_STANDBY | MODE_LONG_RANGE);
    sx_write(REG_FIFO_ADDR_PTR, 0x00);
    sx_write(REG_PAYLOAD_LEN, idx);
    sx_write_fifo(frame, idx);
    Serial.printf("TX fcnt=%lu freq=%.1fMHz SF%d frame: ", fcnt - 1, freq / 1e6, sf);
    for (int i = 0; i < idx; i++) Serial.printf("%02X ", frame[i]);
    Serial.println();
    sx_write(REG_OP_MODE, MODE_TX | MODE_LONG_RANGE);
    unsigned long t0 = millis();
    while (!digitalRead(SX1276_DIO0)) {
        if (millis() - t0 > 5000) {
            Serial.println("TX timeout!");
            return;
        }
    }
    sx_write(REG_IRQ_FLAGS, 0xFF);
    Serial.println("TX done.");
    sx_write(REG_OP_MODE, MODE_STANDBY | MODE_LONG_RANGE);
}

struct TxParams {
    uint32_t freq;
    uint8_t sf;
};

static const TxParams TX_SCHEDULE[] = {
    {868100000UL, 7}, {868300000UL, 7}, {868500000UL, 7}, {868100000UL, 10}, {868300000UL, 10}, {868500000UL, 10},
};

static const int TX_SCHEDULE_LEN = sizeof(TX_SCHEDULE) / sizeof(TX_SCHEDULE[0]);
static int tx_idx = 0;

static uint32_t gap_ms(uint8_t sf) { return TX_INTERVAL_MS; }

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    Serial.println("Raw LoRaWAN TX");
    SPI.begin();
    sx1276_init();
}

void loop() {
    const TxParams& p = TX_SCHEDULE[tx_idx];
    apply_tx_params(p.freq, p.sf);
    uint8_t payload[4];
    payload[0] = (fcnt >> 24) & 0xFF;
    payload[1] = (fcnt >> 16) & 0xFF;
    payload[2] = (fcnt >> 8) & 0xFF;
    payload[3] = fcnt & 0xFF;
    lorawan_send(payload, sizeof(payload), p.freq, p.sf);
    tx_idx = (tx_idx + 1) % TX_SCHEDULE_LEN;
    delay(gap_ms(p.sf));
}

