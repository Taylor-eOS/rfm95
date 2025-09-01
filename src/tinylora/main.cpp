#include <Arduino.h>
#include <SPI.h>
#include <TinyLoRa.h>
#include "secrets.hpp"

#define PIN_SCK 18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_CS 5
#define PIN_RST 14
#define PIN_DIO0 26

TinyLoRa lora = TinyLoRa(PIN_DIO0, PIN_CS, PIN_RST);

void setupSPI() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  setupSPI();
  if (!lora.begin()) {
    Serial.println("TinyLoRa begin failed");
    while (millis() < 2000) delay(10);
  }
  lora.setChannel(MULTI);
  lora.setDatarate(SF7BW125);
  lora.setPower(14);
  unsigned char payload[4] = {0xDE,0xAD,0xBE,0xEF};
  lora.sendData(payload, sizeof(payload), lora.frameCounter);
  Serial.print("FrameCounter sent: ");
  Serial.println(lora.frameCounter);
  lora.frameCounter++;
}

void loop() {
  while (true) {
    delay(1000);
  }
}

