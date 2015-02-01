// RFM12B Sender for DHT22 Sensor with Watchdog (saves much more Power!)
//
// Basiert zum Teil auf der Arbeit von Nathan Chantrell
//
// modified by meigrafd @ 31.01.2015
//------------------------------------------------------------------------------
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include <avr/sleep.h>
#include <DHT22.h>    // https://github.com/nathanchantrell/Arduino-DHT22

// Power-Save-Stuff.
// http://www.surprisingedge.com/low-power-atmegatiny-with-watchdog-timer/
// https://www.sparkfun.com/tutorials/309
// http://jeelabs.org/tag/lowpower/
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
#define NODEID          1   // network ID used for this unit
#define NETWORKID     210   // the network ID we are on
#define GATEWAYID      22   // the node ID we're sending to
#define FREQ  RF12_433MHZ   // Frequency of RFM12B module

#define ACK_TIME     2000   // # of ms to wait for an ack
#define requestACK   false  // request ACK? (true/false)
#define SENDDELAY  300000   // wait this many ms between sending packets. 300000ms / 1000 / 8 = 37,5 * 8sec = 5Min.

//------------------------------------------------------------------------------
// PIN-Konfiguration 
//------------------------------------------------------------------------------
// SENSOR pins
#define DHT22_PIN 10     // DHT sensor is connected on D10 (ATtiny pin 13)
#define DHT22_POWER 9   // DHT Power pin is connected on D9 (ATtiny pin 12)
// LED pin
#define LEDpin 8         // D8, PA2 (ATtiny pin 11) - set to 0 to disable LED
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
//#define KEY   "a4gBM69UZ03lQyK4"

// Setup the DHT
DHT22 myDHT22(DHT22_PIN);

// Variablen für Temperatur/Luftfeuchtigkeit
long temp; long humi;
// Temperatur-String zum Versand per 433 Mhz
char msg[26];

// 7,5 * 8sec = 1 Min. 37,5 * 8sec = 5Min.
int watchdog_limit = SENDDELAY / 1000 / 8;

//##############################################################################

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

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
      ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  //result = 1125300L / result; // Back-Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = 1126400L / result; // Back-calculate Vcc in mV
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
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    bitSet(PRR, PRADC);    // power down the ADC
  }
}

// send ATtiny into Power Save Mode
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

// init Setup
void setup() {
  // configure RFM12B
  rf12_initialize(NODEID, FREQ, NETWORKID);
  #ifdef KEY
    rf12_encrypt((byte*)KEY);      // comment this out to disable encryption
  #endif
  rf12_control(0xC040);   // Adjust low battery voltage to 2.2V
  rf12_sleep(0);           // sleep right away to save power

  pinMode(DHT22_POWER, OUTPUT);  // set power pin for Sensor to output

  // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
  // 6=1sec, 7=2sec, 8=4sec, 9=8sec
  // From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
  setup_watchdog(watchdog_wakeup);      // Wake up after 8 sec
  watchdog_counter = 500; // set to have an initial transmition when starting the sender.

  PRR = bit(PRTIM1);          // only keep timer 0 going
  enableADC(false);           // power down/disable the ADC
  analogReference(INTERNAL);   // Set the aref to the internal 1.1V reference

  if (LEDpin) {
    activityLed(1,1000); // LED on
  }
}

// Loop
void loop() {
  goToSleep(); // goes to sleep for about 8 seconds and continues to execute code when it wakes up

  if (watchdog_counter >= watchdog_limit) {
    watchdog_counter = 1;

    enableADC(true); // power up/enable the ADC

    pinMode(DHT22_POWER, OUTPUT);  // set power pin for Sensor to output
    digitalWrite(DHT22_POWER, HIGH); // turn Sensor on
  
    DHT22_ERROR_t errorCode;
    delay(2500);  // Kurzer Delay zur Initialisierung (DHT22 benoetigt mindestens 2 Sekunden fuer Aufwaermphase nach dem Power-On)
    errorCode = myDHT22.readData(); // read data from sensor
  
    if (errorCode == DHT_ERROR_NONE) { // data is good
      int temp = (myDHT22.getTemperatureC()*100); // Get temperature reading and convert to integer, reversed at receiving end
      int humi = (myDHT22.getHumidity()*100); // Get humidity reading and convert to integer, reversed at receiving end
      int vcc  = readVcc(); // Get supply voltage
      // msg-Variable mit Daten zum Versand fuellen, die spaeter an das WebScript uebergeben werden
      //snprintf(msg, 26, "v=%d&t=%d&h=%d", vcc, temp, humi) ;
      strcpy(msg,"v=");
      itoa(vcc,&msg[strlen(msg)],10);
      strcat(msg,"&t=");
      itoa(temp,&msg[strlen(msg)],10);
      strcat(msg,"&h=");
      itoa(humi,&msg[strlen(msg)],10);
  
      rf12_sleep(-1);  // Wake up RF module
      while (!rf12_canSend())
      rf12_sendStart(GATEWAYID, (uint8_t *)msg, strlen(msg), requestACK);
      rf12_sendWait(2);    //wait for RF to finish sending (2=standby mode, 3=power down)
      rf12_sleep(0);
  
      if (LEDpin) {
        blink(LEDpin, 2); // blink LED
      }
    }
    digitalWrite(DHT22_POWER, LOW); // turn Sensor off to save power
    pinMode(DHT22_POWER, INPUT); // set power pin for Sensor to input before sleeping, saves power
    enableADC(false); // power down/disable the ADC
  }
}

