
#include <Arduino.h>
#include <wireless.h>
#include <HardwareSerial.h>

HardwareSerial &hc12 = Serial1;

void hc12_turn_on() { digitalWrite(PIN_M0, 0); };
void hc12_turn_off() { digitalWrite(PIN_M0, 1); };
void hc12_set_mode() { digitalWrite(PIN_SET0, 0); };
void hc12_normal_mode()
{
    digitalWrite(PIN_SET0, 1);
    delay(200);
};

void print_me()
{
    static char c;
    if (hc12.available())
    {
        while (hc12.available())
        {
            c = hc12.read();
        }
    }
}

void hc12_power_on_setup()
{
    hc12_normal_mode();
    delay(100);

#ifdef FIRSTTIME
    hc12_turn_off();
    delay(100);
    hc12.begin(9600);
    hc12_set_mode();
    hc12_turn_on();
    delay(100);
    while (hc12.available())
        hc12.read();
    hc12.print(check_cmd);
    delay(100);
    print_me();
    hc12.print(channel_cmd);
    delay(100);
    print_me();
    hc12.print(power_cmd);
    delay(100);
    print_me();
    hc12.print(baud_cmd);
    delay(100);
    print_me();
    hc12.print(check_cmd);
    delay(100);
    print_me();
    hc12.print(rx_cmd);
    delay(200);
    print_me();
    hc12_normal_mode();
    delay(400);
    hc12.end();
    delay(100);
    hc12_set_mode();
#endif

    hc12.begin(2400);
    delay(100);
    while (hc12.available())
        hc12.read();

#ifdef FIRSTTIME
    hc12.print(check_cmd);
    delay(100);
    print_me();
    hc12.print(rx_cmd);
    delay(100);
    print_me();
#endif
};

void hc12_setup_hot()
{
    // hc12_turn_off();
    hc12_normal_mode();
    hc12_turn_on();
    delay(1000);
    hc12.begin(2400);
    hc12_set_mode();
};