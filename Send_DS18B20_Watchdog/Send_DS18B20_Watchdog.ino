//--------------------------------------------------------------------------------------
// TempTX-tiny ATtiny84 Based Wireless Temperature Sensor Node
// By Nathan Chantrell http://nathan.chantrell.net
// Updated by Martin Harizanov (harizanov.com) to work with DS18B20
// To use with DS18B20 instead of TMP36, a 4K7 resistor is needed between the Digital 9 and Digital 10 of the ATTiny (Vdd and DQ)
// To get this to compile follow carefully the discussion here: http://arduino.cc/forum/index.php?topic=91491.0
// GNU GPL V3
//--------------------------------------------------------------------------------------
// modified by meigrafd @ 12.01.2015
//------------------------------------------------------------------------------
//#include <JeeLib.h> // https://github.com/jcw/jeelib
#include "pins_arduino.h"
#include <RFM12B.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Power-Save-Stuff.
// http://www.surprisingedge.com/low-power-atmegatiny-with-watchdog-timer/
// https://www.sparkfun.com/tutorials/309
// http://jeelabs.org/tag/lowpower/
#include <avr/sleep.h>
volatile int watchdog_counter;

// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) { watchdog_counter++; }

// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
// From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
int watchdog_wakeup = 9;  // Wake up after 8 sec

//------------------------------------------------------------------------------
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID         1  // RF12 node ID in the range 1-30
#define NETWORKID    210  // RF12 Network group
#define GATEWAYID     22  // the node ID we're sending to
#define FREQ RF12_433MHZ  // Frequency of RFM12B module

#define ACK_TIME    2000  // Number of milliseconds to wait for an ack
#define requestACK false  // request ACK? (true/false)
#define SENDDELAY 300000  // wait this many ms between sending packets. 300000ms / 1000 / 8 = 37,5 * 8sec = 5Min.
//------------------------------------------------------------------------------
// PIN-Konfiguration 
//------------------------------------------------------------------------------
// SENSOR pins
#define ONE_WIRE_BUS 10   // DS18B20 Temperature sensor is connected on D10/ATtiny pin 13
#define ONE_WIRE_POWER 9  // DS18B20 Power pin is connected on D9/ATtiny pin 12
// LED pin
#define LEDpin 8          // D8, PA2 (ATtiny pin 11) - set to 0 to disable LED
//------------------------------------------------------------------------------
/*
                     +-\/-+
               VCC  1|    |14  GND
          (D0) PB0  2|    |13  AREF (D10)
          (D1) PB1  3|    |12  PA1 (D9)
       (PB3) RESET  4|    |11  PA2 (D8)
INT0  PWM (D2) PB2  5|    |10  PA3 (D7)
      PWM (D3) PA7  6|    |9   PA4 (D6) SCK
SDA   PWM (D4) PA6  7|    |8   PA5 (D5) PWM
                     +----+
*/

//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
//#define KEY   "ABCDABCDABCDABCD"

//------------------------------------------------------------------------------

// Need an instance of the Radio Module
RFM12B radio;

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Temperatur-String zum Versand per 433 Mhz
char msg[26];

// 7,5 * 8sec = 1 Min. 37,5 * 8sec = 5Min.
int watchdog_limit = SENDDELAY / 1000 / 8;

//########################################################################################################################

static void activityLed (byte state, byte time = 0) {
  if (LEDpin) {
    pinMode(LEDpin, OUTPUT);
    if (time == 0) {
      digitalWrite(LEDpin, state);
    } else {
      digitalWrite(LEDpin, state);
      delay(time);
      digitalWrite(LEDpin, !state);
    }
  }
}

// blink led
static void blink (byte pin, byte n = 3) {
  if (LEDpin) {
    pinMode(pin, OUTPUT);
    for (byte i = 0; i < 2 * n; ++i) {
      delay(100);
      digitalWrite(pin, !digitalRead(pin));
    }
  }
}

//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
long readVcc() {
   bitClear(PRR, PRADC);
   ADCSRA |= bit(ADEN); // Enable the ADC
   long result;
   // Read 1.1V reference against Vcc
   #if defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
   #else
     ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // For ATmega328
   #endif
   delay(2); // Wait for Vref to settle
   ADCSRA |= _BV(ADSC); // Convert
   while (bit_is_set(ADCSRA,ADSC));
   result = ADCL;
   result |= ADCH<<8;
   result = 1126400L / result; // Back-calculate Vcc in mV
   ADCSRA &= ~ bit(ADEN);
   bitSet(PRR, PRADC); // Disable the ADC to save power
   return result;
}

//--------------------------------------------------------------------------------------------------
// Power Save Functions
//--------------------------------------------------------------------------------------------------

// Enable / Disable ADC, saves ~230uA
void enableADC(bool b) {
  if (b == true){
    bitClear(PRR, PRADC); // power up the ADC
    ADCSRA |= bit(ADEN);  // enable the ADC
    delay(10);
  } else {
    //ADCSRA &= ~(1<<ADEN); // Disable ADC
    ADCSRA &= ~ bit(ADEN);  // disable the ADC
    bitSet(PRR, PRADC);     // power down the ADC
  }
}

void goToSleep() {
  // SLEEP_MODE_IDLE -the least power savings
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN -the most power savings
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode.
  sleep_enable(); // Enable sleep mode.
  sleep_mode(); // Enter sleep mode.

  // After waking from watchdog interrupt the code continues to execute from this point.

  sleep_disable(); // Disable sleep mode after waking.
  // Re-enable the peripherals.
  //power_all_enable();
}

// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
// From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {
  if (timerPrescaler > 9 ) timerPrescaler = 9; //Correct incoming amount if need be
  byte bb = timerPrescaler & 7; 
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary
  //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); //Clear the watchdog reset
  WDTCSR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
  WDTCSR = bb; //Set new watchdog timeout value
  WDTCSR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}

//--------------------------------------------------------------------------------------------------

void setup() {
  radio.Initialize(NODEID,FREQ,NETWORKID); // Initialize RFM12 with settings defined above
  #ifdef KEY
    radio.Encrypt((byte*)KEY);
  #endif
  radio.Control(0xC040);      // Adjust low battery voltage to 2.2V
  radio.Sleep();              // Put the RFM12 to sleep

  watchdog_counter = 500; // set to have an initial transmition when starting the sender.
  setup_watchdog(watchdog_wakeup);

  PRR = bit(PRTIM1);          // only keep timer 0 going
  enableADC(false);           // power down/disable the ADC

  if (LEDpin) {
    activityLed(1,1000); // LED on
  }
}

void loop() {
  goToSleep(); // goes to sleep for about 8 seconds and continues to execute code when it wakes up

  if (watchdog_counter >= watchdog_limit) {
    watchdog_counter = 1;
    
    enableADC(true); // power up/enable the ADC

    pinMode(ONE_WIRE_POWER, OUTPUT); // set power pin for DS18B20 to output
    digitalWrite(ONE_WIRE_POWER, HIGH); // turn DS18B20 sensor on
    delay(50); // Allow 50ms for the sensor to be ready

    // Start up the library
    sensors.begin();
    delay(100); // Allow 100ms for the sensor to be ready
    sensors.requestTemperatures(); // Send the command to get temperatures  
  
    int temp = sensors.getTempCByIndex(0)*100; // read sensor 1
    //int temp2=(sensors.getTempCByIndex(1)*100); // read second sensor.. you may have multiple and count them upon startup but I only need one
  
    digitalWrite(ONE_WIRE_POWER, LOW); // turn Sensor off to save power
    pinMode(ONE_WIRE_POWER, INPUT); // set power pin for Sensor to input before sleeping, saves power

    int supplyV = readVcc(); // Get supply voltage
  
    // msg-Variable mit Daten zum Versand fuellen, die spaeter an das WebScript uebergeben werden
    strcpy(msg,"v=");
    itoa(supplyV,&msg[strlen(msg)],10);
    strcat(msg,"&t=");
    itoa(temp,&msg[strlen(msg)],10);
  
    // Send data via RF
    radio.Wakeup();
    radio.Send(GATEWAYID, (uint8_t *)msg, strlen(msg), requestACK);
    radio.SendWait(2);    //wait for RF to finish sending (2=standby mode)
    radio.Sleep();
    if (LEDpin) {
      blink(LEDpin, 2); // blink LED
    }
    enableADC(false); // power down/disable the ADC
  }
}

