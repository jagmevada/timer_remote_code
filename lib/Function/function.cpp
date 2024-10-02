#include <counter.h>
#include <function.h>

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
    if (digitalRead(RIGHT_BUTTON) == LOW && digitalRead(LEFT_BUTTON) == HIGH)
    {
        uint8_t mins = EEPROM.read(EEPROMADDR);
        delay(10);
        uint8_t seconds = EEPROM.read(EEPROMADDR + 1);
        delay(10);

        counter = ConfigurableDownCounter(mins, seconds);

        sent_data.fields.command = CMD_RESTART;
        // is_paused = true;
    }
    else if (digitalRead(RIGHT_BUTTON) == LOW &&
             digitalRead(LEFT_BUTTON) == LOW)
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
    {
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
    display.printFixedN(0, 2, "hello", STYLE_NORMAL, 2);
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

void wakeup()
{
    // Serial.println(F("SSD1306 allocation failed"));
    // display.begin();
    pinMode(ADC_IN, INPUT); // ADC
    ADC0_Enable();
    hwenable(HWI2C);
    hwenable(HWUART0);
    hwenable(HWUART1);
    setup_display();
    Serial.begin(9600);
    PORTC.INTFLAGS = 0xff;
    PORTA.INTFLAGS = 0xff;
    attachInterrupt(UP_BUTTON, isr_up_button, FALLING);
    attachInterrupt(OK_BUTTON, isr_ok_button, FALLING);
    attachInterrupt(DOWN_BUTTON, isr_down_button, FALLING);
    attachInterrupt(LEFT_BUTTON, isr_left_button, FALLING);
    attachInterrupt(RIGHT_BUTTON, isr_right_button, FALLING);
    // SWUP_INTCTRL = INTERRUPT_EN;
    // SWOK_INTCTRL = INTERRUPT_EN;
    // SWDOWN_INTCTRL = INTERRUPT_EN;
    // SWLEFT_INTCTRL = INTERRUPT_EN;
    // SWRIGHT_INTCTRL = INTERRUPT_EN;
    // attachInterrupt(BMS_BUTTON, isr_bms_button, FALLING);
}