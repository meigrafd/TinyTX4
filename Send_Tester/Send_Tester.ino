// RFM12B Sender 
//
// Sketch to test functionality of Send.
// Sends static integer every 3 Sec. Requests ACK. Without power saving.
//
// modified by meigrafd @ 25.01.2015
//------------------------------------------------------------------------------
#include <RFM12B.h>

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
#define requestACK   true   // request ACK? (true/false)
#define SENDDELAY    3000   // wait this many ms between sending packets.

//------------------------------------------------------------------------------
// PIN-Konfiguration 
//------------------------------------------------------------------------------
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

// Need an instance of the Radio Module
RFM12B radio;

// Temperatur-String zum Versand per 433 Mhz
char msg[26];

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

//--------------------------------------------------------------------------------------------------
// Read current supply voltage (in mV)
//--------------------------------------------------------------------------------------------------
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

// init Setup
void setup() {
  // configure RFM12B
  radio.Initialize(NODEID, FREQ, NETWORKID);
  #ifdef KEY
    radio.Encrypt((byte*)KEY); //comment this out to disable encryption
  #endif
  radio.Control(0xC040); // Adjust low battery voltage to 2.2V
  radio.Sleep();         // sleep right away to save power

  analogReference(INTERNAL);   // Set the aref to the internal 1.1V reference

  if (LEDpin) {
    activityLed(1,1000); // LED on
  }
}

void wakeUp(){}

// Loop
void loop() {
  int supplyV = readVcc(); // Get supply voltage
  int testint = 1337;
  // msg-Variable mit Daten zum Versand fuellen, die spaeter an das WebScript uebergeben werden
  strcpy(msg, "v=");
  itoa(supplyV, &msg[strlen(msg)], 10);
  strcat(msg, "&msg=");
  itoa(testint, &msg[strlen(msg)], 10);

  // Send data via RF
  radio.Wakeup();
  radio.Send(GATEWAYID, (uint8_t *)msg, strlen(msg), requestACK);
  //radio.Send(GATEWAYID, &msg, sizeof(msg), requestACK);
  radio.SendWait(2);    //wait for RF to finish sending (2=standby mode, 3=power down)
  radio.Sleep();

  if (LEDpin) {
    blink(LEDpin, 2); // blink LED
  }
  delay(SENDDELAY);
}

