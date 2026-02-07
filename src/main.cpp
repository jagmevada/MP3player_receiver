#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
typedef ADC_MUXPOS_t adc_0_channel_t;
// ATTINY3224/6 has 3 timers  (TCA, TCB0 & TCB1), TCB1 is used by millis, so
// only TCB0 & TCA available TCA has three Compare and 1 period, so 3 PWM and 1
// Overflow ISR and 3 compare ISR can be generated this code demonstrate TCA0
// CMP0 ISR, and TCB0 CMP ISR. this code also demonstrate RTC 1KHz sleep mode
// running in sleep mode as ULPM, Sleeps for 5seconds then activate CPU for 5sec
// in which TCA or TCB0 will toggle LED on PA7(3) pin at 1Hz.
// Also demonstrate wakeup from sleep with PC0(10) button input and Toggle LED
// on PA7(3).
// ######################### defination ########################################
#define SWINPUT 2
#define ADCPIN 0
#define LDOEN 1
#define LDOENABLE() ditialWrite(LDOEN, 1);
#define LDODISABLE() digitalWrite(LDOEN, 0);
#define LEDON() digitalWrite(3, 1);
#define LEDOFF() digitalWrite(3, 0);
#define SETUPLDO() \
  pinMode(         \
      1,           \
      OUTPUT);  // LDOEN OUT external pulled down and LDO UP when goes HIGH.
#define INTERRUPT_EN 0xB
#define INTERRUPT_DIS 0x8

#define SW2_INTCTRL PORTA.PIN6CTRL
#define SW5_INTCTRL PORTB.PIN2CTRL
#define SW9_INTCTRL PORTA.PIN2CTRL


#define OUTPUT_LEDMASK PIN7_bm
#define OUTPUT_PINMASK PIN5_bm

#define DEBOUNCE_PERIOD 5     /// Multiple of 128ms
#define AUTOOFF_TIMEOUT 2445  // 2445  // 3130  Multiple of 128ms or 100ms
#define debouncedelay 100     // ms (user requested define)
#define RTC_TICK_MS 128       // RTC PIT period in ms
#define DEBOUNCE_TICKS 5
#define LED_MS_PIN2 20
#define LED_MS_PIN5 15
#define LED_MS_PIN9 10
#define ADC_PERIOD 156        // 6        // 20s at multiple of 128ms
#define BAT_LOW_TIMEOUT 7
#define UTH 3300
#define LTH 2800
#define LOW 0
#define HIGH 1


// --------- EDIT THESE TO MATCH YOUR WIRING/VOLTAGE ---------
const uint8_t PIN_TX_DATA = 8;   // to E160-TxMS1 DATA (3.3V logic!)
const unsigned long T_US = 270;  // base clock unit (µs), 300–400us is typical


const uint32_t  KEY2   = 0xa0a0a0a0;     
const uint32_t  KEY3   = 0xb0b0b0b0;    
const uint32_t  KEY1   = 0xc0c0c0c0;   

// const uint32_t  KEY2   = 0xa5a5a5a0;     
// const uint32_t  KEY3   = 0xb5b5b5b0;    
// const uint32_t  KEY1   = 0xc5c5c5c0; 

// const uint32_t  KEY2   = 0x696969a0;     
// const uint32_t  KEY3   = 0x7a7a7a70;    
// const uint32_t  KEY1   = 0x8b8b8b80; 

// const uint32_t  KEY2   = 0x30303030;     
// const uint32_t  KEY3   = 0x41414140;    
// const uint32_t  KEY1   = 0x52525250;   

// const uint32_t  KEY2   = 0x50505050;     
// const uint32_t  KEY3   = 0x61616160;    
// const uint32_t  KEY1   = 0x72726270; 

// -----------------------------------------------------------



// ######################### variables ########################################
bool new_input, batstate = 1, ldostate = 0, ldostateprev = 0;
bool new_input2, new_input5, new_input9=0;
unsigned int vbat = 4095, vbatx = 4095;
unsigned char t0 = DEBOUNCE_PERIOD, t2 = 0;
uint16_t t1 = ADC_PERIOD;
uint16_t timeout = 0;
boolean txready = 0;
uint8_t which_button = 0;
// Button/LED state (debounce + LED scheduling)
volatile uint8_t debounced_time2 = 0, debounced_time5 = 0, debounced_time9 = 0;
volatile uint8_t led_ticks_remaining = 0; // in RTC ticks (128ms each)
// ######################### function defs ####################################
void init_variable();
void Timer_A0();
void button2_isr();
void button5_isr();
void button9_isr();
void setup_timer_a();
void setup_timer_b0();
void setup_timer_rtc();
void ADC0_StartConversion(adc_0_channel_t channel);
void ADC0_Enable(void);
void ADC0_Disable(void);
void ADC0_Initialize(void);
void presleep();
  void toggleLED();


  //  ##########################remote functions##########################
static inline void outHi(unsigned long us) { digitalWrite(PIN_TX_DATA, HIGH); delayMicroseconds(us); }
static inline void outLo(unsigned long us) { digitalWrite(PIN_TX_DATA, LOW);  delayMicroseconds(us); }
void sendSync();
void sendBit(bool one);
void sendFrame(uint32_t key4) ;

// // ######################### presleep() #########################
void presleep() {
  ADC0_Disable();
  Serial1.end();
}

// // ######################### init variables #########################
void init_variable() {
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
void ADC0_StartConversion(adc_0_channel_t channel) {
  ADC0.MUXPOS &= ADC_VIA_gm;
  ADC0.MUXPOS |= channel;  // VDDDIV10
  ADC0.COMMAND &= ~ADC_DIFF_bm;
  ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
}

// ####################### adc_enable function  ######################
void ADC0_Enable(void) { ADC0.CTRLA |= ADC_ENABLE_bm; }
// ####################### adc_disable function  ######################
void ADC0_Disable(void) { ADC0.CTRLA &= ~ADC_ENABLE_bm; }

// ####################### adc_initializaion function  ######################

void ADC0_Initialize(void) {
  // PRESC System clock divided by 64;
  ADC0.CTRLB = 0xF;
  // FREERUN disabled; LEFTADJ disabled; SAMPNUM No accumulation;
  ADC0.CTRLF = 0x0;
  // REFSEL Internal 2.048V Reference; TIMEBASE 1;
  ADC0.CTRLC = 0xD;
  // WINCM No Window Comparison; WINSRC RESULT;
  ADC0.CTRLD = 0x0;
  // SAMPDUR 255;
  ADC0.CTRLE = 0xFF;  // CLK_ADC
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
void Timer_A0(void) {  // 100ms
}
// ######################### button_input #############################



// ######################### setup_timer_a0 #############################
void setup_timer_a() {
  takeOverTCA0();
  TCA0_SINGLE_PER = 10000;  // PER/F_CPU, 10ms at 1MHz
  TCA0_SINGLE_CMP0 = 2500;  // intterupt at CMP0<TOP, TOP=PER;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc |  // F_CPU * DIV1
                      TCA_SINGLE_ENABLE_bm;        // TCA_SINGLE_RUNSTDBY_bm;
  TCA0_SINGLE_INTFLAGS =
      TCA_SINGLE_CMP0_bm;  // Clear Any pending interrupt flags
  TCA0_SINGLE_INTCTRL = TCA_SINGLE_CMP0_bm;  // Enable TCA0 timeout interrupt
};
// ######################### setup_timer_b0 #############################
void setup_timer_b0() {
  TCB0.CCMP = 10000;                                   // CCMP/1MHz, 10ms
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;  //| TCB_RUNSTDBY_bm;
  TCB0.INTFLAGS = TCB_CAPT_bm;  // Clear any pending interrupt flags
  TCB0.INTCTRL = TCB_CAPT_bm;   // Enable TCB0 timeout interrupt
};
// ######################### setup_timer_rtc #############################
void setup_timer_rtc() {
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;  // 1khz internal
  RTC.CTRLA = CLKCTRL_RUNSTDBY_bm | RTC_RTCEN_bm |
              RTC_PRESCALER0_bm;  // run in standby sleep mode and enable rtc no
                                  // prescaler
  RTC.PITCTRLA = RTC_PERIOD_CYC128_gc | RTC_PITEN_bm;  // every 128msec interrupt
  RTC.PITINTCTRL = RTC_PI_bm;                          // enable interrupt
}
//@@@@@@@@@@@@@@@@@@@@@@@@@  SETUP  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void setup() {
  init_variable();
  pinMode(0, OUTPUT);  // ADC
  digitalWrite(0, LOW);
  SETUPLDO();
  LDODISABLE();
  pinMode(2, INPUT_PULLUP);  // SW IN using internal pullup (button to GND)
  pinMode(3, OUTPUT);
  LEDOFF();
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(PIN_TX_DATA, OUTPUT);
  digitalWrite(PIN_TX_DATA, LOW);  // idle low per our waveform builder
  pinMode(9, INPUT_PULLUP);  // CHARGE IN OPTIONAL, NEED EX. PULLDOWN AND IT
                             // WILL RISE UP WHEN USB CONNECTED
  pinMode(10, INPUT_PULLUP);
  batstate = HIGH;
  new_input = 0;
  presleep();
  ADC0_Initialize();
  ADC0_Disable();
  set_sleep_mode(SLEEP_MODE_STANDBY);
  // Attach dedicated async interrupts for pins 2,5,9 (FALLING edge, active low button)
  attachInterrupt(2, button2_isr, FALLING);
  attachInterrupt(5, button5_isr, FALLING);
  attachInterrupt(9, button9_isr, FALLING);
  setup_timer_rtc();
  sleep_enable();
  sei();  // Enable global interrupts
}

//@@@@@@@@@@@@@@@@@@@@@@@@@  LOOP  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void loop() {
if(which_button){
  if(which_button==1)
    sendFrame(KEY1);
    else if(which_button==2)
      sendFrame(KEY2);
      else if(which_button==3)
        sendFrame(KEY3);
      else ;
      which_button=0;
  }else{
    sleep_cpu();
  }
}

  void toggleLED(){  
      if((PORTA.OUT & OUTPUT_LEDMASK)==0){
        PORTA.OUT |= OUTPUT_LEDMASK;
      }else{
        PORTA.OUT &= ~OUTPUT_LEDMASK;
      }
  }

// ************************* ISR Timer RTC ***************************
ISR(RTC_PIT_vect) {  /// 128 msec timer

  if (new_input2) {   /// routine for enable button interrupt after debounce
    debounced_time2--;            // debounce timer countdown
    if (debounced_time2 == 0) {   // button debounce period over
      new_input2 = 0;
      SW2_INTCTRL = INTERRUPT_EN;  // enable interrupt
      PORTA.INTFLAGS = 0x40;   // clear previous flags
    }
  }
  
  if (new_input5) {   /// routine for enable button interrupt after debounce
    debounced_time5--;            // debounce timer countdown
    if (debounced_time5 == 0) {   // button debounce period over
      new_input5 = 0;
      SW5_INTCTRL = INTERRUPT_EN;  // enable interrupt
      PORTB.INTFLAGS = 0x04;  // PB2 // clear INTFLAG
    }
  }

  if (new_input9) {   /// routine for enable button interrupt after debounce
    debounced_time9--;            // debounce timer countdown
    if (debounced_time9 == 0) {   // button debounce period over
      new_input9 = 0;
      SW9_INTCTRL = INTERRUPT_EN;  // enable interrupt
      PORTB.INTFLAGS = 0x04;  // PA2 // clear INTFLAG
    }
  }



  // // LED on-duration countdown
  // if (led_ticks_remaining > 0) {
  //   led_ticks_remaining--;
  //   if (led_ticks_remaining == 0) {
  //     // turn LED off when time elapsed
  //     LEDOFF();
  //   }
  // }

  if (timeout > 0) {  // Normal autoff timeout countdown
    timeout--;
    if (timeout == 0) {
      presleep();
    }
  }

  if (0) {  // Vbat monitoring and auto cutoff
    if (t1 == 0) {  // adc timing
      t1 = ADC_PERIOD;
      vbatx = vbat;  // Copy previous conversoin result
      ADC0_StartConversion(ADC_MUXPOS_AIN4_gc);
    }
    t1--;

    if (batstate == HIGH) {  // battery is discharging to cutoff
      if (vbatx < LTH) {     // reached at cutoff
        batstate = LOW;
        timeout = 0;
        presleep();
      }
    } else {  // charging to reach full charge voltage 4.2V
      if (vbatx > UTH) {
        batstate = HIGH;
      }
    }
  } else {
    sleep_enable();
  }

  RTC.PITINTFLAGS = RTC_PI_bm; /* Clear the interrupt flag */
}


// ************************* Button ISRs ***************************
// These ISRs are tiny and only set flags and start debounce timer.
void button2_isr() {
  which_button=1;
  // led_ticks_remaining = LED_MS_PIN2;
  // LEDON();
  new_input2 = true; // Indicate new input detected
  debounced_time2 = DEBOUNCE_TICKS;
  SW2_INTCTRL = INTERRUPT_DIS;
  PORTA.INTFLAGS = 0x40;  // PA6 // clear INTFLAG
}

void button5_isr() {
  which_button=2;
  // led_ticks_remaining = LED_MS_PIN5;
  // LEDON();
  new_input5 = true; // Indicate new input detected
  debounced_time5 = DEBOUNCE_TICKS;
  SW5_INTCTRL = INTERRUPT_DIS;
  PORTB.INTFLAGS = 0x04;  // PB2 // clear INTFLAG
}

void button9_isr() {
  which_button=3;
  // led_ticks_remaining = LED_MS_PIN9;
  // LEDON();
  new_input9 = true; // Indicate new input detected
  debounced_time9 = DEBOUNCE_TICKS;
  SW9_INTCTRL = INTERRUPT_DIS;
  PORTA.INTFLAGS = 0x04;  // PA2 // clear INTFLAG
}

// ************************* ISR ADC Result ready  ***************************
ISR(ADC0_RESRDY_vect) {
  /* Insert your ADC result ready interrupt handling code here */
  vbat = (uint16_t)ADC0.RESULT;
  txready = 1;
  /* The interrupt flag has to be cleared manually */
  ADC0.INTFLAGS = ADC_RESRDY_bm;
}
// ************************* ISR Timer A0 ***************************
ISR(TCA0_CMP0_vect) {  /// 10ms timer  // active only when awake
  TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm; /* Clear the interrupt flag */
}
// ************************* ISR Timer B0 ***************************
ISR(TCB0_INT_vect) {           ///  // active only when awake
  TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
}






// 1527 symbols per docs:
// 1 => 3T high + 1T low
// 0 => 1T high + 3T low
// SYNC => 4T high + 124T low
void sendBit(bool one) {
  if (one) { outHi(3*T_US); outLo(1*T_US); }
  else     { outHi(1*T_US); outLo(3*T_US); }
}

void sendSync() { outHi(4*T_US); outLo(124*T_US); }
void sendFrame(uint32_t key4) {
  // SYNC
  sendSync();
  for (int i = 31; i >= 0; --i) {
    sendBit((key4 >> i) & 1);
  }
}
