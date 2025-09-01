#include <Arduino.h>
#include <SPI.h>

#define PIN_CS 5
#define PIN_RST 14
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK 18

SPISettings spiSettings(8000000, MSBFIRST, SPI_MODE0);

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

void printHexDec(const char* label, uint8_t v) {
  Serial.print(label);
  Serial.print(" 0x");
  if (v < 16) Serial.print("0");
  Serial.print(v, HEX);
  Serial.print(" (");
  Serial.print(v, DEC);
  Serial.println(")");
}

bool testWriteAndRead(uint8_t writeVal, uint8_t mask) {
  writeRegister(0x01, writeVal);
  delay(50);
  uint8_t r = readRegister(0x01);
  uint8_t maskedRead = r & mask;
  uint8_t maskedWrite = writeVal & mask;
  Serial.print("Wrote 0x");
  if (writeVal < 16) Serial.print("0");
  Serial.print(writeVal, HEX);
  Serial.print(" (");
  Serial.print(writeVal, DEC);
  Serial.print(") Read 0x");
  if (r < 16) Serial.print("0");
  Serial.print(r, HEX);
  Serial.print(" (");
  Serial.print(r, DEC);
  Serial.print(") MaskedRead 0x");
  if (maskedRead < 16) Serial.print("0");
  Serial.print(maskedRead, HEX);
  Serial.print(" Expected 0x");
  if (maskedWrite < 16) Serial.print("0");
  Serial.print(maskedWrite, HEX);
  Serial.print(" => ");
  if (maskedRead == maskedWrite) {
    Serial.println("OK");
    return true;
  } else {
    Serial.println("FAIL");
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("FRM95 Mode switch diagnostic starting");
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  resetChip();
  uint8_t version = readRegister(0x42);
  printHexDec("Version register:", version);
  if (version != 0x12) {
    Serial.println("Warning: unexpected version; continuing tests anyway");
  } else {
    Serial.println("Version OK");
  }
  Serial.println("Initial OpMode:");
  printHexDec("OpMode", readRegister(0x01));
}

void loop() {
  static uint32_t lastCycle = 0;
  if (millis() - lastCycle < 2000) return;
  lastCycle = millis();
  Serial.println();
  Serial.print("Cycle at ms ");
  Serial.println(millis());
  uint8_t mask = 0x87;
  bool allok = true;
  allok &= testWriteAndRead(0x80, mask);
  allok &= testWriteAndRead(0x81, mask);
  allok &= testWriteAndRead(0x85, mask);
  allok &= testWriteAndRead(0x80, mask);
  Serial.print("Mode switch cycle result: ");
  Serial.println(allok ? "ALL OK" : "SOME FAIL");
  Serial.print("Current OpMode raw: ");
  uint8_t cur = readRegister(0x01);
  if (cur < 16) Serial.print("0");
  Serial.println(cur, HEX);
  Serial.print("OpMode bits (LRMode+Mode): 0x");
  uint8_t curbits = cur & mask;
  if (curbits < 16) Serial.print("0");
  Serial.println(curbits, HEX);
}

