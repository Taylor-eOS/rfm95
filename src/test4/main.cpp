#include <Arduino.h>
#include <SPI.h>

#define PIN_CS 5
#define PIN_RST 14
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK 18
#define PIN_DIO0 26

SPISettings spiSettings(8000000, MSBFIRST, SPI_MODE0);

const uint8_t REG_FIFO = 0x00;
const uint8_t REG_OP_MODE = 0x01;
const uint8_t REG_FRF_MSB = 0x06;
const uint8_t REG_FRF_MID = 0x07;
const uint8_t REG_FRF_LSB = 0x08;
const uint8_t REG_PA_CONFIG = 0x09;
const uint8_t REG_FIFO_ADDR_PTR = 0x0D;
const uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
const uint8_t REG_PAYLOAD_LENGTH = 0x22;
const uint8_t REG_IRQ_FLAGS = 0x12;

uint8_t readRegister(uint8_t addr) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void writeRegister(uint8_t addr, uint8_t val) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void resetChip() {
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);
  digitalWrite(PIN_RST, HIGH);
  delay(20);
}

void setFrequency(uint32_t freqHz) {
  uint64_t frf = ((uint64_t)freqHz << 19) / 32000000;
  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void enterLoRaStandby() {
  writeRegister(REG_OP_MODE, 0x81);
  delay(10);
}

bool sendTestPacket(const uint8_t *data, uint8_t len, uint16_t timeoutMs) {
  uint8_t txBase = readRegister(REG_FIFO_TX_BASE_ADDR);
  writeRegister(REG_FIFO_ADDR_PTR, txBase);
  for (uint8_t i = 0; i < len; ++i) writeRegister(REG_FIFO, data[i]);
  writeRegister(REG_PAYLOAD_LENGTH, len);
  writeRegister(REG_IRQ_FLAGS, 0xFF);
  writeRegister(REG_OP_MODE, 0x83);
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    uint8_t irq = readRegister(REG_IRQ_FLAGS);
    if (irq & 0x08) {
      writeRegister(REG_IRQ_FLAGS, 0xFF);
      return true;
    }
    delay(5);
  }
  return false;
}

void printHexDec(const char* label, uint8_t v) {
  Serial.print(label);
  Serial.print(" 0x");
  if (v < 16) Serial.print("0");
  Serial.print(v, HEX);
  Serial.print(" (");
  Serial.print(v, DEC);
  Serial.println(")");
}

void setup() {
  Serial.begin(115200);
  delay(50);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DIO0, INPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  resetChip();
  uint8_t version = readRegister(0x42);
  printHexDec("Version register:", version);
  enterLoRaStandby();
  setFrequency(868000000);
  writeRegister(REG_PA_CONFIG, 0x8F); // PA_BOOST, about +14 dBm
  Serial.println("Frequency set to 868 MHz, PA_BOOST enabled");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 6000) return;
  last = millis();
  Serial.println();
  Serial.print("Cycle at ms ");
  Serial.println(millis());
  uint8_t pa = readRegister(REG_PA_CONFIG);
  printHexDec("PaConfig:", pa);
  uint8_t irqBefore = readRegister(REG_IRQ_FLAGS);
  printHexDec("IRQ flags before:", irqBefore);
  uint8_t pkt[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  bool ok = sendTestPacket(pkt, 4, 3000);
  Serial.print("Tx attempt result: ");
  Serial.println(ok ? "TxDone" : "TIMEOUT");
  uint8_t irqAfter = readRegister(REG_IRQ_FLAGS);
  printHexDec("IRQ flags after:", irqAfter);
}

