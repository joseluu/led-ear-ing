#include "FastLED.h"
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/io.h>


// Utility macros
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

#define DEBUG 0
// How many leds in your strip?
#if DEBUG
#define NUM_LEDS 1
#else
#define  NUM_LEDS 8
#endif

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 3
#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];


void setup() { 
  //clock_prescale_set(clock_div_4);
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  pinMode(4, OUTPUT);  // ring power

#if DEBUG
  pinMode(0, OUTPUT);  // debug on
  pinMode(2, OUTPUT);  // debug off
 #endif
  adc_disable(); // ADC uses ~320uA
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void enterSleep(void)
{
  sleep_enable();
  sleep_cpu();
}

boolean on;
unsigned char ran;
void flash(){
   static int i=0;
    if (on) {
     ran= random8();

      // Turn the LED on, then pause
      digitalWrite(0, HIGH);   // strobe debug
      digitalWrite(4, HIGH);   // lights on !
#if DEBUG
      i=0;
#else
      i=ran&7;
#endif      
      digitalWrite(0, LOW);    // end of debug strobe
      pinMode(3, OUTPUT);      // enable led programming
      switch ((ran &0x18)>>3){
 #define MAX 31
        case 0:
          leds[i]=CRGB(MAX,MAX,MAX);
          break;
        case 1:
          leds[i]=CRGB(MAX,0,0);
          break;
        case 2:
         leds[i]=CRGB(0,MAX,0);
          break;
        case 3:
         leds[i]=CRGB(0,0,MAX);
          break;
      }
      FastLED.show();    // send programming to leds
      pinMode(3, INPUT);  // to save current

   } else {
      // Now turn the LED off, then pause
      #if DEBUG
            digitalWrite(2, HIGH);
            digitalWrite(2, LOW);
      #endif
      pinMode(3, OUTPUT);
      leds[i] = CRGB::Black;
       FastLED.show();
       digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      pinMode(3, INPUT);
   }
   on = !on;
}

void waitSleep(int millisec) {
  while (millisec) {
    goSleep(on);
    millisec--;
  }
}

void goSleep(boolean fast) {
  watchdogSetup(fast); //enable watchDog
  power_timer0_disable(); //disable Timer 0
  power_timer1_disable(); //disable Timer 1
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  //disable watchdog after sleep
  wdt_disable();
  power_timer0_enable(); //enable Timer 0
  power_timer1_enable(); //enable Timer 1
}

void watchdogSetup(boolean fast) {

  //WDP3 - WDP2 - WPD1 - WDP0 - time
  // 0      0      0      0      16 ms
  // 0      0      0      1      32 ms
  // 0      0      1      0      64 ms
  // 0      0      1      1      0.125 s
  // 0      1      0      0      0.25 s
  // 0      1      0      1      0.5 s
  // 0      1      1      0      1.0 s
  // 0      1      1      1      2.0 s
  // 1      0      0      0      4.0 s
  // 1      0      0      1      8.0 s


  // Reset the watchdog reset flag
  bitClear(MCUSR, WDRF);
  // Start timed sequence
  bitSet(WDTCR, WDCE); //Watchdog Change Enable to clear WD
  bitSet(WDTCR, WDE); //Enable WD

if (!fast) {
    bitClear(WDTCR, WDP3);
    bitClear(WDTCR, WDP2);
    bitClear(WDTCR, WDP1);
    bitClear(WDTCR, WDP0);  
} else {
#if DEBUG
  // Set new watchdog timeout value to 64ms
  //bitSet(WDTCR, WDP1); //64ms
  bitSet(WDTCR, WDP2); //250ms
#else
  if (ran & 0x1) {
    bitSet(WDTCR, WDP3);
  } else {
    bitSet(WDTCR, WDP2);
    if (ran & 0x4)
      bitSet(WDTCR, WDP1); 
  }
  if (ran & 0x2)
    bitSet(WDTCR, WDP0); 
#endif
}

  // Enable interrupts instead of reset
  bitSet(WDTCR, WDIE);
}

ISR(WDT_vect) {
  // Don't do anything here but we must include this
  // block of code otherwise the interrupt calls an
  // uninitialized interrupt handler.
}


void loop() { 
  flash();
  
  waitSleep(1);

}

