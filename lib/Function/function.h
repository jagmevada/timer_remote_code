#pragma once
#include <Arduino.h>
#include <wireless.h>
#include <EEPROM.h>
#include "lcdgfx.h"
#include "lcdgfx_gui.h"
#include <Wire.h>
// ###################################################################
// ###################### DEFINED CONSTANTS ##########################
// ###################################################################
#define ADC_IN 0
#define POWEREN 1
#define SW6 2 // INT
#define VCCEN 3
#define LDOEN 4
#define SET 5
#define PC_RX 6
#define PC_TX 7
#define SDA 8
#define SCL 9
#define SW1 10
#define SW2 11
#define SW3 12 // INT
#define SW5 13
#define HC_TX 14
#define HC_RX 15
#define SW4 16
// ALIAS
#define SW_UP SW1
#define SW_DOWN SW3
#define SW_LEFT SW4
#define SW_RIGHT SW5
#define SW_OK SW2
#define SW_POWER SW6
#define SWINPUT SW_POWER
#define HWUART0 1
#define HWUART1 2
#define HWI2C 3

#define SETUPLDO() pinMode(LDOEN, OUTPUT);
#define LDOENABLE() digitalWrite(LDOEN, 0);  // active high, dcdc LDO on
#define LDODISABLE() digitalWrite(LDOEN, 0); // active high, dcdc ldo shutdown
#define SETUPPOWER() pinMode(POWEREN, OUTPUT);
#define POWERENABLE() digitalWrite(POWEREN, 0);  // active low,  VIN INPUT TO DCDC LDO supply UNavailable,
#define POWERDISABLE() digitalWrite(POWEREN, 1); // active low,  VIN INPUT TO DCDC LDO supply available,
#define SETUPVAUX() pinMode(VCCEN, OUTPUT);
#define VAUXENABLE() digitalWrite(VCCEN, 0);  // active low, I2C/UART1 module supply available
#define VAUXDISABLE() digitalWrite(VCCEN, 0); // active low, I2C, UART1 module supply unavailable
#define SETUPUART0() pinMode(PC_TX, OUTPUT);
#define INTERRUPT_EN 0x3
#define INTERRUPT_DIS 0x0
#define SW_INTCTRL PORTA.PIN6CTRL /// POWER_SW interrupt enabled
#define SWUP_INTCTRL PORTC.PIN0CTRL
#define SWOK_INTCTRL PORTC.PIN1CTRL
#define SWDOWN_INTCTRL PORTC.PIN2CTRL
#define SWLEFT_INTCTRL PORTA.PIN3CTRL
#define SWRIGHT_INTCTRL PORTC.PIN3CTRL

#define OUTPUT_PINMASK PIN5_bm
#define DEBOUNCE_PERIOD 5    /// Multiple of 128ms
#define AUTOOFF_TIMEOUT 2445 // 2445  // 3130  Multiple of 128ms or 100ms
#define ADC_PERIOD 16        // 6        // 20s at multiple of 128ms
#define BAT_LOW_TIMEOUT 7
#define UTH 3300
#define LTH 2800
#define LOW 0
#define HIGH 1

#define CMD_RESTART 0
#define CMD_RESUME 1
#define CMD_HOLD 2
#define CMD_SET_RESUME 3 // Will send payload
#define CMD_STATUS 4
#define CMD_NEWDATA 5
#define CMD_EEPROM 6            //
#define CMD_NONE2 7             //
#define CHARGEPIN PIN_PA7       // 3
#define BATTERY_SENSPIN PIN_PA6 // 2
#define LEFT_BUTTON SW_LEFT     // 1  // S4
#define RIGHT_BUTTON SW_RIGHT   // 13  // S5
#define UP_BUTTON SW_UP         // 10  // S1
#define OK_BUTTON SW_OK         // 11  // S2
#define DOWN_BUTTON SW_DOWN     // 12  // S3
#define BMS_BUTTON \
    PIN_PB5 // battery management system button, for battery only
#define EXTRA_PIN PIN_PB4

#define OLED_SW PIN_PB3

// Analog to digital converter time?
#define ADCTIME 20 // multiply by 100ms
// Sync time?
#define SYNC_THRESHOLD 10         // multiply by 100ms
#define BMS_TRIGGER_THRESHOLD 100 // multiply by 100ms
#define EEPROMADDR 8

// ###################################################################
// ###################### HARDWARE INITIALIZATION ####################
// ###################################################################

// ###################################################################
// ###################### DATA TYPES #################################
// ###################################################################

typedef uint8_t u8;
typedef uint16_t u16;
typedef ADC_MUXPOS_t adc_0_channel_t;

typedef struct
{
    u16 parity : 1;  // 1-bit field for parity
    u16 address : 4; // 4-bit field for address
    u16 command : 3; // 3-bit field for command
} protocol_byte;

typedef union
{
    u8 byte;
    protocol_byte fields;
} protocol_data;

// ####################################################################
// ###################### FUNCTIONS DECLARATION #######################
// ####################################################################
u8 paritycheck(u8 data);
void setup_io();
void isr_up_button();
void isr_down_button();
void isr_ok_button();
void isr_right_button();
void isr_left_button();
void setup_display();
void update_display_state();
void wakeup();
extern void hwenable(u8 hwid);
extern void hwdisable(u8 hwid);
extern void ADC0_Enable();
