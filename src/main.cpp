
// ATTINY3224/6 has 3 timers  (TCA, TCB0 & TCB1), TCB1 is used by millis, so
// only TCB0 & TCA available TCA has three Compare and 1 period, so 3 PWM and 1
// Overflow ISR and 3 compare ISR can be generated this code demonstrate TCA0
// CMP0 ISR, and TCB0 CMP ISR. this code also demonstrate RTC 1KHz sleep mode
// running in sleep mode as ULPM, Sleeps for 5seconds then activate CPU for 5sec
// in which TCA or TCB0 will toggle LED on PA7(3) pin at 1Hz.
// Also demonstrate wakeup from sleep with PC0(10) button input and Toggle LED
// on PA7(3).

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <function.h>

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
#define VAUXDISABLE() digitalWrite(VCCEN, 1); // active low, I2C, UART1 module supply unavailable
#define SETUPUART0() pinMode(PC_TX, OUTPUT);
#define INTERRUPT_EN 0x3
#define INTERRUPT_DIS 0x0
#define SW_INTCTRL PORTA.PIN6CTRL /// POWER_SW interrupt enabled
#define OUTPUT_PINMASK PIN5_bm
#define DEBOUNCE_PERIOD 5    /// Multiple of 128ms
#define AUTOOFF_TIMEOUT 2445 // 2445  // 3130  Multiple of 128ms or 100ms
#define ADC_PERIOD 16        // 6        // 20s at multiple of 128ms
#define BAT_LOW_TIMEOUT 7
#define UTH 3300
#define LTH 2800
#define LOW 0
#define HIGH 1

// ###################################################################
// ###################### HARDWARE INITIALIZATION ####################
// ###################################################################

// LORA_E32 lora(Serial1, M0, M1, AUX);

// ###################################################################
// ###################### DATA TYPES #################################
// ###################################################################

typedef uint8_t u8;
typedef uint16_t u16;
typedef ADC_MUXPOS_t adc_0_channel_t;

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
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
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
    pinMode(6, INPUT_PULLUP);
    pinMode(7, INPUT_PULLUP); // TX
    // digitalWrite(7, 1);
    break;
  case 2: // UART1
    pinMode(15, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP); // TX
    // digitalWrite(14, 1);
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
  pinMode(ADC_IN, OUTPUT); // ADC pin output low for power saving
  digitalWrite(ADC_IN, LOW);
  SETUPLDO();
  LDODISABLE();
  SETUPPOWER();
  POWERDISABLE();
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

  hwdisable(HWUART0);
  hwdisable(HWUART1);
  hwdisable(HWI2C);

  batstate = HIGH;
  new_input = 0;
  Serial.begin(9600);
  presleep();
  ADC0_Initialize();
  ADC0_Disable();
  set_sleep_mode(SLEEP_MODE_STANDBY);
  attachInterrupt(SWINPUT, button_input, FALLING);
  setup_timer_rtc();
  sleep_enable();
  sei(); // Enable global interrupts
}

// ###################################################################
// #######################  LOOP#  ###################################
// ###################################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@  LOOP  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void loop()
{

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

void presleep()
{
  Serial.println("sleeping");
  PORTA.OUT &= ~OUTPUT_PINMASK; // turn off LDO bat low
  pinMode(ADC_IN, OUTPUT);      // ADC
  digitalWrite(ADC_IN, LOW);
  ldostateprev = 1;
  ldostate = 0;
  ADC0_Disable();
  Serial.end();
  hwdisable(HWUART0);
  hwdisable(HWUART1);
  hwdisable(HWI2C);
  // pinMode(9, INPUT_PULLUP);
}

// // ######################### init variables #########################
void init_variable()
{
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
void button_input(void)
{
  // if(new_input==0){
  t0 = DEBOUNCE_PERIOD;
  SW_INTCTRL = INTERRUPT_DIS;
  // ldostateprev = ldostate;
  ldostate = !ldostate;
  new_input = 1;
  if (ldostate)
  {                              // LDO to be turned on
    PORTA.OUT |= OUTPUT_PINMASK; // turn on again but
    ADC0_Enable();
    t1 = 3;
    timeout = AUTOOFF_TIMEOUT;
    pinMode(ADC_IN, INPUT); // ADC
    hwenable(HWI2C);
    hwenable(HWUART0);
    hwenable(HWUART1);
    Serial.begin(9600);
    Serial.println("Wokeup");
    POWERENABLE();
    LDOENABLE();
    // VAUXENABLE();
  }
  else
  { // LDO to be turned off

    presleep();
    LDODISABLE();
    POWERDISABLE();
    // VAUXDISABLE();
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
void setup_timer_b0()
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
{                              ///  // active only when awake
  TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
}