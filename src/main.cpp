// this is new timer's remote controller code, major coded added from your repo, all buttons are working, power on/off working as expected
// RF interface not tested, you have to include RF code for initialization (FIRST TIME) for programming HC12 by seting SET PIN low and then running normal code (repraggming by commenting FIRST TIME),
// all button are working remote get pausing and timer time setting working but not coming out of pause, may be required RF side interface
// timer b0 ISR runnting at 10ms in active (normal) mode, all cases are commented in new code and added as required so kindly add transmitter code
// Serial for debug and Serial1 for hc12 comms.
// power button main action: when pressed- mcu calls presleep to disable all ios and go to sleep mode, when pressed again it calls wakeup to reinit ios.
// when sleeping it cut-off power from OLED display and hc12, when wakup it given power to both, howerver both need to reinialize again, OLED initialization incorporated already, HC12 to be done by u

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <EEPROM.h>
#include <SPI.h>

// The project fails to build without the above two includes.
#include <function.h>
#include <counter.h>
#include <wireless.h>
#include "lcdgfx.h"
#include "lcdgfx_gui.h"
// LORA_E32 lora(Serial1, M0, M1, AUX);

// extern DisplaySSD1306_128x64_I2C display(-1);

DisplaySSD1306_128x64_I2C display(-1);
extern volatile bool update_display;
extern volatile bool is_paused;
extern ConfigurableDownCounter counter;
extern void isr_up_button();
extern void isr_ok_button();
extern void isr_down_button();
extern void isr_left_button();
extern void isr_right_button();
// ####################################################################
// ###################### FUNCTIONS DECLARATION #######################
// ####################################################################
void init_variable();
void Timer_A0();
void button_input();
void setup_timer_a();
void setup_timer_b0();
void setup_timer_rtc();
void ADC0_StartConversion(adc_0_channel_t channel);
void ADC0_Enable(void);
void ADC0_Disable(void);
void ADC0_Initialize(void);
void presleep();
void hwdisable(u8 hardware);
void hwenable(u8 hardware);

// ###################################################################
// ############## GLOBAL VARIABLES ###################################
// ###################################################################

bool new_input, batstate = 1, ldostate = 0, ldostateprev = 0;
unsigned int vbat = 4095, vbatx = 4095;
unsigned char t0 = DEBOUNCE_PERIOD, t2 = 0;
uint16_t t1 = ADC_PERIOD;
uint16_t timeout = 0;
boolean txready = 0;
// ###################################################################
// #######################  SETUP  ###################################
// ###################################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@  SETUP  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void setup()
{
  init_variable();
  setup_io();

  hwdisable(HWUART0);
  hwdisable(HWUART1);
  hwenable(HWI2C);
  delay(1000);

  batstate = HIGH;
  new_input = 0;
  Serial.begin(9600);
  // presleep();
  ADC0_Initialize();
  ADC0_Enable();
  presleep();
  POWERDISABLE();
  set_sleep_mode(SLEEP_MODE_STANDBY);
  attachInterrupt(SWINPUT, button_input, FALLING);
  setup_timer_rtc();
  setup_timer_b0();
  sleep_enable();
  sei(); // Enable global interrupts
}

// void setup()
// {
//   init_variable();
//   void setup_io();
//   // SETUPLDO();
//   // LDOENABLE();
//   // PORTA.OUT |= OUTPUT_PINMASK; // turn on again but
//   pinMode(ADC_IN, INPUT); // ADC
//   ADC0_Enable();
//   t1 = 3;
//   timeout = AUTOOFF_TIMEOUT;
//   hwenable(HWI2C);
//   hwenable(HWUART0);
//   hwenable(HWUART1);
//   Serial.begin(9600);
//   Serial.println("Wokeup");
//   POWERENABLE();
//   delay(1000);
//   setup_display();
//   delay(1000);
//   // hwdisable(HWUART0);
//   // hwdisable(HWUART1);
//   // hwenable(HWI2C);
//   batstate = HIGH;
//   new_input = 0;
//   Serial.begin(9600);
//   Serial.println("init..ed");

//   // presleep();
//   ADC0_Initialize();
//   ADC0_Disable();
//   set_sleep_mode(SLEEP_MODE_STANDBY);
//   attachInterrupt(SWINPUT, button_input, FALLING);
//   setup_timer_rtc();
//   setup_timer_b0();
//   // hc12_power_on_setup();
//   // hc12_normal_mode();

//   // // Serial.begin(2400);
//   // update_eeprom_display = true;
//   // sleep_enable();
//   sei(); // Enable global interrupts
// }

// ###################################################################
// #######################  LOOP#  ###################################
// ###################################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@  LOOP  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void loop()
{
  if (update_display)
  {
    update_display_state();
    update_display = false;
  }
  // ADC0.CTRLA &= ~ADC_ENABLE_bm;
  if (txready)
  {
    Serial.println(vbat * 4.096 / 4096);
    txready = 0;
  }
  if (ldostate == 0)
  {
    sleep_cpu();
  }
}
// ###################################################################
// #######################  FUNCTIONS DEFINATION #####################
// ###################################################################

void hwenable(u8 hardware)
{
  switch (hardware)
  {
  case 1: // UART0
    pinMode(6, INPUT_PULLUP);
    pinMode(7, OUTPUT); // TX
    digitalWrite(7, HIGH);
    break;
  case 2: // UART1
    pinMode(15, INPUT_PULLUP);
    pinMode(14, OUTPUT); // TX
    digitalWrite(14, HIGH);
    break;
  case 3: // I2C
    pinMode(8, INPUT);
    pinMode(9, INPUT);
    break;
  default:
    break;
  }
}

void hwdisable(u8 hardware)
{
  switch (hardware)
  {
  case 1: // UART0
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT); // TX
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    break;
  case 2: // UART1
    pinMode(15, OUTPUT);
    pinMode(14, OUTPUT); // TX
    digitalWrite(14, LOW);
    digitalWrite(15, LOW);
    break;
  case 3: // I2C
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    break;
  default:
    break;
  }
}

void presleep()
{
  // Serial.println("sleeping");
  // Serial.println(BOD.CTRLA, HEX);
  // Serial.println(BOD.CTRLB, HEX);

  display.end();
  pinMode(ADC_IN, OUTPUT); // ADC
  digitalWrite(ADC_IN, LOW);
  ldostateprev = 1;
  ldostate = 0;
  ADC0_Disable();
  Serial.end();
  hwdisable(HWUART0);
  hwdisable(HWUART1);
  hwdisable(HWI2C);

  SWUP_INTCTRL = INTERRUPT_DIS;
  SWOK_INTCTRL = INTERRUPT_DIS;
  SWDOWN_INTCTRL = INTERRUPT_DIS;
  SWLEFT_INTCTRL = INTERRUPT_DIS;
  SWRIGHT_INTCTRL = INTERRUPT_DIS;

  detachInterrupt(UP_BUTTON);
  detachInterrupt(OK_BUTTON);
  detachInterrupt(DOWN_BUTTON);
  detachInterrupt(LEFT_BUTTON);
  detachInterrupt(RIGHT_BUTTON);

  // pinMode(9, INPUT_PULLUP);
}

// // ######################### init variables #########################
void init_variable()
{
  // BOD.CTRLA = 0x2;
  // BOD.CTRLB = 0x0;
#ifdef FIRSTTIME
  EEPROM.write(EEPROMADDR, 20);
  delay(10);
  EEPROM.write(EEPROMADDR + 1, 0);
  delay(10);
#endif

  // uint8_t mins = EEPROM.read(EEPROMADDR);
  // delay(10);
  // uint8_t seconds = EEPROM.read(EEPROMADDR + 1);
  // delay(10);
  // counter = ConfigurableDownCounter(mins, seconds);

  new_input = 0;
  batstate = 1;
  ldostate = 0;
  ldostateprev = 0;
  vbat = 4095;
  vbatx = 4095;
  t0 = DEBOUNCE_PERIOD;
  t2 = 0;
  t1 = 9;
  timeout = 0;
  txready = 0;
}

// // ######################### adc_startconv function #########################
void ADC0_StartConversion(adc_0_channel_t channel)
{
  ADC0.MUXPOS &= ADC_VIA_gm;
  ADC0.MUXPOS |= channel; // VDDDIV10
  ADC0.COMMAND &= ~ADC_DIFF_bm;
  ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
}

// ####################### adc_enable function  ######################
void ADC0_Enable(void) { ADC0.CTRLA |= ADC_ENABLE_bm; }
// ####################### adc_disable function  ######################
void ADC0_Disable(void) { ADC0.CTRLA &= ~ADC_ENABLE_bm; }

// ####################### adc_initializaion function  ######################

void ADC0_Initialize(void)
{
  // PRESC System clock divided by 64;
  ADC0.CTRLB = 0xF;
  // FREERUN disabled; LEFTADJ disabled; SAMPNUM No accumulation;
  ADC0.CTRLF = 0x0;
  // REFSEL Internal 2.048V Reference; TIMEBASE 1;
  ADC0.CTRLC = 0xD;
  // WINCM No Window Comparison; WINSRC RESULT;
  ADC0.CTRLD = 0x0;
  // SAMPDUR 255;
  ADC0.CTRLE = 0xFF; // CLK_ADC
  // ADCPGASAMPDUR 6 ADC cycles; GAIN 1X Gain; PGABIASSEL 1x BIAS current; PGAEN
  // disabled;
  ADC0.PGACTRL = 0x0;
  // DBGRUN disabled;
  ADC0.DBGCTRL = 0x0;
  // DIFF disabled; MODE SINGLE_12BIT; START Stop an ongoing conversion;
  ADC0.COMMAND = 0x10;
  // RESOVR disabled; RESRDY enabled; SAMPOVR disabled; SAMPRDY disabled;
  // TRIGOVR disabled; WCMP disabled;
  ADC0.INTCTRL = 0x1;
  // MUXPOS ADC input pin 4; VIA Via ADC;
  ADC0.MUXPOS = 0x4;
  // MUXNEG Ground; VIA Via ADC;
  ADC0.MUXNEG = 0x30;
  // Window comparator high threshold
  ADC0.WINHT = 0x0;
  // Window comparator low threshold
  ADC0.WINLT = 0x0;
  // ENABLE disabled; LOWLAT disabled; RUNSTDBY disabled;
  ADC0.CTRLA = 0x0;
}

// ######################### timer_a0 isr function #############################
void Timer_A0(void)
{ // 100ms
}
// ######################### button_input #############################
void button_input(void) /// power button, Toggle to OFF (deep sleep) and ON (countdown or normal ops)
{
  // if(new_input==0){
  t0 = DEBOUNCE_PERIOD;
  SW_INTCTRL = INTERRUPT_DIS;
  // ldostateprev = ldostate;
  ldostate = !ldostate;
  new_input = 1;
  if (ldostate)
  { // LDO to be turned on
    POWERENABLE();

    t1 = 3;
    timeout = AUTOOFF_TIMEOUT;
    wakeup();
    // Serial.println("Wokeup");
    // Serial.println(BOD.CTRLA, HEX);
    // Serial.println(BOD.CTRLB, HEX);
  }
  else
  { // LDO to be turned off
    presleep();
    POWERDISABLE();
  }
  //}
  PORTA.INTFLAGS = 0x40; // PA6 // clear INTFLAG
  //    SLPCTRL.CTRLA = 0x3; // enable sleep and go to standby mode
}

// ######################### setup_timer_a0 #############################

/// @brief Use cautiously as it disables millis()
void setup_timer_a()
{
  takeOverTCA0();
  TCA0_SINGLE_PER = 10000;                        // PER/F_CPU, 10ms at 1MHz
  TCA0_SINGLE_CMP0 = 2500;                        // intterupt at CMP0<TOP, TOP=PER;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | // F_CPU * DIV1
                      TCA_SINGLE_ENABLE_bm;       // TCA_SINGLE_RUNSTDBY_bm;
  TCA0_SINGLE_INTFLAGS =
      TCA_SINGLE_CMP0_bm;                   // Clear Any pending interrupt flags
  TCA0_SINGLE_INTCTRL = TCA_SINGLE_CMP0_bm; // Enable TCA0 timeout interrupt
};
// ######################### setup_timer_b0 #############################
void setup_timer_b0() // 10ms
{
  TCB0.CCMP = 10000;                                  // CCMP/1MHz, 10ms
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //| TCB_RUNSTDBY_bm;
  TCB0.INTFLAGS = TCB_CAPT_bm;                        // Clear any pending interrupt flags
  TCB0.INTCTRL = TCB_CAPT_bm;                         // Enable TCB0 timeout interrupt
};
// ######################### setup_timer_rtc #############################
void setup_timer_rtc()
{
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // 1khz internal
  RTC.CTRLA = CLKCTRL_RUNSTDBY_bm | RTC_RTCEN_bm |
              RTC_PRESCALER0_bm;                      // run in standby sleep mode and enable rtc no
                                                      // prescaler
  RTC.PITCTRLA = RTC_PERIOD_CYC128_gc | RTC_PITEN_bm; // every 2 sec interrupt
  RTC.PITINTCTRL = RTC_PI_bm;                         // enable interrupt
}

// ************************* ISR Timer RTC ***************************
ISR(RTC_PIT_vect)
{ /// 128 msec timer
  if (new_input)
  {       /// routine for enable button interrupt after debounce
          /// period
    t0--; // debounce timer countdown
    if (t0 == 0)
    { // button debounce period over
      new_input = 0;
      SW_INTCTRL = INTERRUPT_EN; // enable interrupt
      PORTA.INTFLAGS = PIN6_bm;  // clear previous flags
    }
  }

  if (timeout > 0)
  { // Normal autoff timeout countdown, set to AUTOOFF_TIMEOUT
    // when power button pressed
    timeout--;
    if (timeout == 0)
    { // Normal autoff timeout event
      presleep();
    }
  }

  if (ldostate == 1)
  { // Vbat monitoring and auto cutoff

    if (t1 == 0)
    { // adc timing
      t1 = ADC_PERIOD;
      vbatx = vbat; // Copy previous conversoin result
      ADC0_StartConversion(ADC_MUXPOS_AIN4_gc);
    }
    t1--;

    if (batstate == HIGH)
    { // battery is discharging to cutoff
      if (vbatx < LTH)
      { // reached at cutoff
        batstate = LOW;
        timeout = 0;
        presleep();
      }
    }
    else
    { // charging to reach full charge voltage 4.2V
      if (vbatx > UTH)
      {
        batstate = HIGH;
      }
    }
  }
  else
  {
    sleep_enable();
  }
  RTC.PITINTFLAGS = RTC_PI_bm; /* Clear the interrupt flag */
}

// ************************* ISR ADC Result ready  ***************************
ISR(ADC0_RESRDY_vect)
{
  /* Insert your ADC result ready interrupt handling code here */
  vbat = (uint16_t)ADC0.RESULT;
  txready = 1;
  /* The interrupt flag has to be cleared manually */
  ADC0.INTFLAGS = ADC_RESRDY_bm;
}
// ************************* ISR Timer A0 ***************************
ISR(TCA0_CMP0_vect)
{                                            /// 10ms timer  // active only when awake
  TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm; /* Clear the interrupt flag */
}
// ************************* ISR Timer B0 ***************************

ISR(TCB0_INT_vect)
{
  static uint32_t timer_count = 0;
  static uint8_t bat_adc_time = ADCTIME;
  static uint8_t oled_synctime = SYNC_THRESHOLD;

  // TL;DR: each case is executed every 100ms
  //
  // After the first round of timer_count increments,
  // each case is executed every 100ms (The ISR is invoked
  // every 10ms, at which point ONLY ONE switch case will be
  // executed, and timer_count will be incremented, essentially
  // doing a round-robin among the cases every 10ms, and coming
  // back to the same case every 10ms * NUM_SWITCH_CASES(10 in this case) =
  // 100ms)
  switch (timer_count)
  {
  case 0:
  {
    break;
  }
  case 2:
  {
    break;
  }
  case 4:
  {
    break;
  }
  case 6:
  {
    // if (wait_for_acknowledgement)
    // {
    //   if (hc12.available())
    //   {
    //     recieve_data.byte = hc12.read();
    //     if (recieve_data.byte == sent_data.byte)
    //     {
    //       switch (sent_data.fields.command)
    //       {
    //       case CMD_RESUME: // 1
    //         // mon.print("command RESUME received\n");
    //         break;
    //       case CMD_HOLD: // 2
    //         // mon.print("command HOLD received\n");
    //         is_paused = true;
    //         update_display = true;
    //         break;
    //       case CMD_RESTART:    // 0
    //       case CMD_SET_RESUME: // 3
    //         // mon.print("command SET received\n");
    //         is_paused = false;
    //         break;
    //       case CMD_STATUS: // 4
    //         // mon.print("command STATUS received\n");
    //         break;
    //       case CMD_NEWDATA: // 5
    //         // mon.print("command NEWDATA received\n");
    //         break;
    //       case CMD_EEPROM: // 6
    //         is_paused = false;
    //         break;
    //       default:
    //         // mon.print("command %d not implemented\n",
    //         // data.fields.command);
    //         break;
    //       }
    //     }
    //   }
    // }
    break;
  }
  // 10*10 ms round up
  case 10:
  {
    timer_count = 0;
    if (not is_paused)
    {
      --oled_synctime;
      if (oled_synctime == 0)
      {
        update_display = true;
        counter.CountDownByOneSecond();
        oled_synctime = SYNC_THRESHOLD;
      }
    }
    break;
  }
  default:
  {
    break;
  }
  }

  ++timer_count;

  // Clear the interrupt flag
  TCB0.INTFLAGS = TCB_CAPT_bm;
}