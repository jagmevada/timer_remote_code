#pragma once

// #define FIRSTTIME

#include <Arduino.h>

#define MYADDRESS 1
#define RF_CH (10 + MYADDRESS)
#define DEFAULTBAUD 9600
#define SLOWBAUD 2400
#define TIMERADDR 12 // Max 1-15
#define POWER 8

const String channel_cmd = String("AT+C0") + String(RF_CH);
const String power_cmd = String("AT+P") + String(POWER);
const String baud_cmd = String("AT+B") + String(SLOWBAUD);
const String check_cmd = String("AT");
const String rx_cmd = String("AT+RX");

#define HC_SW PIN_PA3
#define HC_SET PIN_PA4

#define PIN_SET0 HC_SET
#define PIN_M0 HC_SW

void hc12_turn_on();
void hc12_turn_off();
void hc12_set_mode();
void hc12_normal_mode();
void print_me();
void hc12_power_on_setup();
void hc12_setup_hot();
