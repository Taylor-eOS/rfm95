#ifndef DIAGNOSTICS_HPP
#define DIAGNOSTICS_HPP
#include <Arduino.h>
#include <lmic.h>
#include <SPI.h>

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

extern uint16_t tx_success_count;
extern uint16_t tx_fail_count;
uint8_t readRegister(uint8_t address);
void writeRegister(uint8_t address,uint8_t value);
void resetChip();
void setup_eu868_channels();
void print_device_info();
void print_channel_status();
void print_lmic_status();
void print_transmission_stats();

#endif

