// RFM12B Repeater
//
// Relays from one Network Group to another maintaining the Node ID
//
// Avoid using any code with interrupts (e.g powersaving watchdogs)
// as it seems to cause resets and the setup code to execute again
// Power Saving doesn't really make sense for a Relay as it needs to be listening all the time
// and I would expect it to be running from a mains PSU or similar.
//
// modified by meigrafd @ 06.01.2015
//------------------------------------------------------------------------------
#include <RFM12B.h>

//------------------------------------------------------------------------------

// Enable SoftwareSerial for debugging?
#define DEBUG 1
#define SERIAL_BAUD     9600

//------------------------------------------------------------------------------
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255

// Receive settings - where Senders send to
#define NODEID         22   // network ID used for this unit
#define NETWORKID     210   // the network ID we are on
#define FREQ  RF12_433MHZ   // Frequency of RFM12B module
#define ACK_TIME     2000   // # of ms to wait for an ack
#define requestACK   false   // request ACK? (true/false)

// Transmit settings - where Repeater sends to PI
#define T_NODEID          1   // network ID used for this unit
#define T_NETWORKID     220   // the network ID we are on
#define T_GATEWAYID      22   // the node ID we're sending to
#define T_FREQ  RF12_433MHZ   // Frequency of RFM12B module
#define T_ACK_TIME     2000   // # of ms to wait for an ack
#define T_requestACK   false   // request ACK? (true/false)

// UART pins - only for DEBUG
#define rxPin 7 // D7, PA3
#define txPin 3 // D3, PA7. pin der an RXD vom PI geht.

// LED pin
#define LEDpin 8 // D8, PA2 - set to 0 to disable LED
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
#define KEY   "a4gBM69UZ03lQyK4"

// Need an instance of the Radio Module
RFM12B radio;

#ifdef DEBUG
  #include <SoftwareSerial.h>
  // Initialise UART
  SoftwareSerial mySerial(rxPin, txPin);
#endif
//##############################################################################

static void activityLED (byte state, byte time = 0) {
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

void enableADC(bool b) {
  if (b == true){
    bitClear(PRR, PRADC); // power up the ADC
    ADCSRA |= bit(ADEN); // enable the ADC
    delay(10);
  } else {
    //ADCSRA &= ~(1<<ADEN); // Disable ADC, saves ~230uA
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    bitSet(PRR, PRADC); // power down the ADC
  }
}

//--------------------------------------------------------------------------------------------------
// Read current supply voltage (in mV)
//--------------------------------------------------------------------------------------------------
long readVcc() {
  enableADC(true);
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
  enableADC(false);
  return result;
}

//--------------------------------------------------------------------------------------------------

// Initialise the RFM12B for receiving
void setupReceive() {
  radio.Initialize(NODEID, FREQ, NETWORKID);
  #ifdef KEY
    radio.Encrypt((byte*)KEY); //comment this out to disable encryption
  #endif
}

// Initialise the RFM12B for relaying
void setupTransmit() {
  radio.Initialize(T_NODEID, T_FREQ, T_NETWORKID);
  #ifdef KEY
    radio.Encrypt((byte*)KEY); //comment this out to disable encryption
  #endif
}

// init Setup
void setup() {
  #ifdef DEBUG
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    mySerial.begin(SERIAL_BAUD);
  #endif
  setupReceive();
  radio.Control(0xC040); // Adjust low battery voltage to 2.2V

  enableADC(false); // power down/disable the ADC
  //ACSR = (1<<ACD); // Disable the analog comparator
  //DIDR0 = 0x3F; // Disable digital input buffers on all ADC0-ADC5 pins.
  //ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
  PRR = bit(PRTIM1); // only keep timer 0 going
  analogReference(INTERNAL);   // Set the aref to the internal 1.1V reference
  if (LEDpin) {
    activityLED(1,1000); // LED on
  }
}

// Loop
void loop() {
  if (radio.ReceiveComplete()) {
    if (radio.CRCPass()) {
      int supplyV = readVcc(); // Get supply voltage
  
      //node ID of TX, extracted from RF datapacket. Not transmitted as part of structure
      byte theNodeID = radio.GetSender();
      #ifdef DEBUG
        mySerial.print(theNodeID, DEC);
        mySerial.print(" ");
      #endif

      // Buffer to store incoming data
      const char* inData;
      int i;
      for (byte i = 0; i < *radio.DataLen; i++) { //can also use radio.GetDataLen() if you don't like pointers
        #ifdef DEBUG
          mySerial.print((char) radio.Data[i]);
        #endif
        inData += (char)radio.Data[i];
      }

      if (radio.ACKRequested()) {
        radio.SendACK();
        #ifdef DEBUG
          mySerial.print(" - ACK sent");
        #endif
      }
  
      // Send received data to PI
      setupTransmit();
      radio.Send(T_GATEWAYID, (uint8_t *)inData, strlen(inData), requestACK);
      radio.SendWait(2);    //wait for RF to finish sending (2=standby mode, 3=power down)
      setupReceive();

      if (LEDpin) {
        blink(LEDpin, 2);
      }
    } else {
      #ifdef DEBUG
        mySerial.print("BAD-CRC");
      #endif
      if (LEDpin) {
        activityLED(1); // LED on
        delay(1000);
        activityLED(0); // LED off
      }
    }
    #ifdef DEBUG
      mySerial.println();
    #endif
  }
}

