// RFM12B Sender for DHT22 Sensor
//
// Basiert zum Teil auf der Arbeit von Nathan Chantrell
//
// modified by meigrafd @ 16.12.2013
//------------------------------------------------------------------------------
#include <RFM12B.h>
#include <avr/sleep.h>
#include <DHT22.h>    // https://github.com/nathanchantrell/Arduino-DHT22
//------------------------------------------------------------------------------
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID         20   // network ID used for this unit
#define NETWORKID     210   // the network ID we are on
#define GATEWAYID      22   // the node ID we're sending to (22). Set to 0 for broadcast to all
#define ACK_TIME     2000   // # of ms to wait for an ack
#define requestACK   false   // request ACK? (true/false)

#define SENDDELAY   300000  // wait this many ms between sending packets. 300000ms = 5min

//#define SENDDELAY 5  // wait this many Minutes between sending packets
//------------------------------------------------------------------------------
// PIN-Konfiguration 
//------------------------------------------------------------------------------
// SENSOR pins
#define DHT22_PIN 10     // DHT sensor is connected on D10 (ATtiny pin 13)
#define DHT22_POWER 9    // DHT Power pin is connected on D9 (ATtiny pin 12)
// LED pin
#define LEDpin 0         // D8, PA2 (ATtiny pin 11) - set to 0 to disable LED
//------------------------------------------------------------------------------
/*
                     +-\/-+
               VCC  1|    |14  GND
          (D0) PB0  2|    |13  AREF (D10)
          (D1) PB1  3|    |12  PA1 (D9)
             RESET  4|    |11  PA2 (D8)
INT0  PWM (D2) PB2  5|    |10  PA3 (D7)
      PWM (D3) PA7  6|    |9   PA4 (D6)
      PWM (D4) PA6  7|    |8   PA5 (D5) PWM
                     +----+
*/

//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
//#define KEY   "ABCDABCDABCDABCD"

// Need an instance of the Radio Module
RFM12B radio;

// Setup the DHT
DHT22 myDHT22(DHT22_PIN);

// Variablen für Temperatur/Luftfeuchtigkeit
long temp; long humi;
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

// init Setup
void setup() {
  pinMode(DHT22_POWER, OUTPUT);  // set power pin for Sensor to output
  radio.Initialize(NODEID, RF12_433MHZ, NETWORKID);
  #ifdef KEY
    radio.Encrypt((byte*)KEY);      //comment this out to disable encryption
  #endif
  // configure RFM12B
  radio.Control(0xC040);   // Adjust low battery voltage to 2.2V
  radio.Sleep(0); //sleep right away to save power
  ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
  PRR = bit(PRTIM1); // only keep timer 0 going
  analogReference(INTERNAL);   // Set the aref to the internal 1.1V reference
  if (LEDpin) {
    activityLed(1,1000); // LED on
  }
}

// Loop
void loop() {
  //activityLed(1);  // LED on
  pinMode(DHT22_POWER, OUTPUT);  // set power pin for Sensor to output
  digitalWrite(DHT22_POWER, HIGH); // turn Sensor on

  bitClear(PRR, PRADC); // power up the ADC
  ADCSRA |= bit(ADEN); // enable the ADC

  DHT22_ERROR_t errorCode;
  delay(2500);  // Kurzer Delay zur Initialisierung (DHT22 benoetigt mindestens 2 Sekunden fuer Aufwaermphase nach dem Power-On)
  errorCode = myDHT22.readData(); // read data from sensor
  //activityLed(0);  // LED off

  if (errorCode == DHT_ERROR_NONE) { // data is good
    int temp = (myDHT22.getTemperatureC()*100); // Get temperature reading and convert to integer, reversed at receiving end
    int humi = (myDHT22.getHumidity()*100); // Get humidity reading and convert to integer, reversed at receiving end
    int vcc  = readVcc(); // Get supply voltage
    // msg-Variable mit Daten zum Versand fuellen, die spaeter an das WebScript uebergeben werden
    //snprintf(msg, 26, "v=%d&t=%d&h=%d", vcc,temp,humi) ;
    strcpy(msg,"v=");
    itoa(vcc,&msg[strlen(msg)],10);
    strcat(msg,"&t=");
    itoa(temp,&msg[strlen(msg)],10);
    strcat(msg,"&h=");
    itoa(humi,&msg[strlen(msg)],10);

    radio.Wakeup();
    radio.Send(GATEWAYID, (uint8_t *)msg, strlen(msg), requestACK);
    radio.SendWait(2);    //wait for RF to finish sending (2=standby mode, 3=power down)
    radio.Sleep(0);

    if (LEDpin) {
      blink(LEDpin, 2); // blink LED
    }
  }

  digitalWrite(DHT22_POWER, LOW); // turn Sensor off to save power
  pinMode(DHT22_POWER, INPUT); // set power pin for Sensor to input before sleeping, saves power

  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC

//  int DELAY = (SENDDELAY*60*1000);  // delay: <min> * 60sec * 1000ms
//  delay(DELAY);
  delay(SENDDELAY);
}

