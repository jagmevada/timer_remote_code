#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include "counter.hpp"
#include "lcdgfx.h"
#include "lcdgfx_gui.h"
#include "wireless.hpp"
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
#define AUTOOFF_TIMEOUT 2445 // 2445 // 2445  // 3130  Multiple of 128ms or 100ms
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

// LORA_E32 lora(Serial1, M0, M1, AUX);
extern DisplaySSD1306_128x64_I2C display;
ConfigurableDownCounter counter;
unsigned char charging_on = 0;
uint8_t battery_level;
char cbat[15];

char down_counter_output[10];
volatile bool is_paused = false;
volatile uint8_t position_to_update = 0;

volatile bool charger_state = false;
volatile unsigned char button_state = 0;
volatile bool button_update = false;
volatile bool update_count = false;
volatile uint16_t adc_reading = 0;
volatile unsigned long t_charge = 0;
volatile bool update_charger = true;
volatile bool update_display = false;
volatile bool update_battery_sense = false;
volatile unsigned char BMS_time = 2;
volatile bool data_available_on_serial = false;
char rx_data;
char crx_data[4];

volatile bool need_to_send = false;
volatile u8 command_to_send = 6;

protocol_data sent_data;

volatile bool wait_for_acknowledgement = false;
protocol_data recieve_data;

volatile bool update_eeprom_display = false;

u8 paritycheck(u8 data)
{
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 1;
}

void isr_up_button()
{
    Serial.println("up");
    if (is_paused)
    {
        update_display = true;
        switch (position_to_update)
        {
        case 0:
        {
            counter.IncrementTensMinuteDigit();
            break;
        }
        case 1:
        {
            counter.IncrementOnesMinuteDigit();
            break;
        }
        case 2:
        {
            counter.IncrementTensSecondDigit();
            break;
        }
        case 3:
        {
            counter.IncrementOnesSecondDigit();
            break;
        }
        default:
        {
        }
        }
    }
};

void isr_down_button()
{
    Serial.println("down");
    if (is_paused)
    {
        update_display = true;
        switch (position_to_update)
        {
        case 0:
        {
            counter.DecrementTensMinuteDigit();
            break;
        }
        case 1:
        {
            counter.DecrementOnesMinuteDigit();
            break;
        }
        case 2:
        {
            counter.DecrementTensSecondDigit();
            break;
        }
        case 3:
        {
            counter.DecrementOnesSecondDigit();
            break;
        }
        default:
        {
        }
        }
    }
};

void isr_ok_button()
{
    Serial.println("ok");
    need_to_send = true;

    sent_data.fields.address = MYADDRESS;
    sent_data.fields.parity = 0;

    // Reset
    if (digitalRead(RIGHT_BUTTON) == LOW && digitalRead(LEFT_BUTTON) == HIGH) // RELOAD TIME FROM EEPROM
    {
        Serial.println("reset");
        uint8_t mins = EEPROM.read(EEPROMADDR);
        delay(10);
        uint8_t seconds = EEPROM.read(EEPROMADDR + 1);
        delay(10);

        counter = ConfigurableDownCounter(mins, seconds);

        sent_data.fields.command = CMD_RESTART;
        // is_paused = true; // comment this code
    }
    else if (digitalRead(RIGHT_BUTTON) == LOW &&
             digitalRead(LEFT_BUTTON) == LOW) //// WRITE NEW TIME TO EEPROM
    {
        // Write to EEPROM and start from there.
        unsigned char mins = (unsigned char)counter.GetMinutes();
        unsigned char seconds = (unsigned char)counter.GetSeconds();

        EEPROM.write(EEPROMADDR, mins);
        delay(10);
        EEPROM.write(EEPROMADDR + 1, seconds);
        delay(10);
        update_eeprom_display = true;

        sent_data.fields.command = CMD_EEPROM;
    }
    else
    { // PAUSE TIMER
        if (is_paused)
        {
            // press should resume
            sent_data.fields.command = CMD_SET_RESUME;
        }
        else
        {
            // press should pause
            sent_data.fields.command = CMD_HOLD;
        }
    }

    sent_data.fields.parity = paritycheck(sent_data.byte);
};

void isr_right_button()
{
    Serial.println("right");
    if (is_paused)
    {
        update_display = true;
        position_to_update = min(position_to_update + 1, 3);
    }
};

void isr_left_button()
{
    Serial.println("left");
    if (is_paused)
    {
        update_display = true;
        position_to_update = max(position_to_update - 1, 0);
    }
};

void setup_display()
{
    display.clear();
    lcd_delay(10);
    display.setFixedFont(ssd1306xled_font6x8);
    lcd_delay(10);
    display.clear();
    lcd_delay(100);
    display.begin();
    lcd_delay(100);
    display.clear();
    lcd_delay(100);
    display.printFixedN(0, 2, "JSCA", STYLE_NORMAL, 2);
};

void update_display_state()
{
    counter.SetOutputString(down_counter_output);
    display.printFixedN(0, 2, down_counter_output, STYLE_NORMAL, 2);

    if (is_paused)
    {
        char marker[6];
        switch (position_to_update)
        {
        case 0:
            snprintf(marker, 6, "^    ");
            break;
        case 1:
            snprintf(marker, 6, " ^   ");
            break;
        case 2:
            snprintf(marker, 6, "   ^ ");
            break;
        case 3:
            snprintf(marker, 6, "    ^");
            break;
        default:
            snprintf(marker, 6, "^^^^^");
            break;
        }
        display.printFixedN(0, 34, marker, STYLE_NORMAL, 2);
    }
    else
    {
        display.printFixedN(0, 34, "     ", STYLE_NORMAL, 2);
    }

    // battery display
    display.printFixedN(0, 51, cbat, STYLE_NORMAL, 0);

    // dispaly address
    {
        static char address_display[7];
        snprintf(address_display, 7, "add:%d", MYADDRESS);
        display.printFixedN(82, 51, address_display, STYLE_NORMAL, 0);
    }

    // EEPROM display
    {
        static uint8_t mins = 0;
        static uint8_t seconds = 0;
        if (update_eeprom_display)
        {
            mins = EEPROM.read(EEPROMADDR);
            delay(10);
            seconds = EEPROM.read(EEPROMADDR + 1);
            delay(10);
            update_eeprom_display = false;
        }
        static char display_eeprom[17];
        snprintf(display_eeprom, 17, "eeprom : %d%d:%d%d", mins / 10, mins % 10,
                 seconds / 10, seconds % 10);
        display.printFixedN(0, 58, display_eeprom, STYLE_NORMAL, 0);
    }
}

void setup_io()
{
    pinMode(ADC_IN, OUTPUT); // ADC pin output low for power saving
    digitalWrite(ADC_IN, LOW);
    SETUPLDO();
    LDODISABLE();
    SETUPPOWER();
    POWERENABLE();
    SETUPVAUX();
    VAUXDISABLE();
    // pinMode(1, OUTPUT); // POWER_EN
    pinMode(2, INPUT); // SW6 POWER_SW external pulled up
    // pinMode(3, OUTPUT); // VCCEN
    // pinMode(4, OUTPUT); // LDO_EN
    // digitalWrite(3, LOW); // it is external pulled down pinMode
    pinMode(5, OUTPUT); // pulled low
    digitalWrite(5, 0);
    // pinMode(6, INPUT_PULLUP); // PC_RX
    // pinMode(7, INPUT_PULLUP); // PC_TX
    // pinMode(8, OUTPUT);        // SDA
    // pinMode(9, OUTPUT);        // SCL
    pinMode(10, INPUT); // SW1
    pinMode(11, INPUT); // SW2
    pinMode(12, INPUT); // SW3
    pinMode(13, INPUT); // SW5
    // pinMode(14, INPUT_PULLUP); // HC_TX//Cautious as this pin is TXD
    // pinMode(15, INPUT_PULLUP); // //Cautious as this pin is RXD
    pinMode(16, INPUT); // SW4

    SWUP_INTCTRL = INTERRUPT_DIS;
    SWOK_INTCTRL = INTERRUPT_DIS;
    SWDOWN_INTCTRL = INTERRUPT_DIS;
    SWLEFT_INTCTRL = INTERRUPT_DIS;
    SWRIGHT_INTCTRL = INTERRUPT_DIS;
};
