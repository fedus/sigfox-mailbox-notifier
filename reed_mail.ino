/*
  SigFox mailbox notifier
  - based on the SigFox Event Trigger tutorial
  
  Two reed switches are used to detect whether mail is inserted into the mailbox
  or whether it is emptied (ie whether the small slit is opened or the whole
  front panel to take mail out)
*/

#include <SigFox.h>
#include <ArduinoLowPower.h>

// Set debug to false to enable continuous mode
// and disable serial prints
int debug = false;

bool got_mail = false;
const int REED_MAIL = 1; // Pin connected to reed switch
const int REED_CLEAR = 0; // Pin connected to reed switch
const int LED_PIN = LED_BUILTIN; // LED pin - active-high

// Structure we are going to send over SigFox, which basically consists of a
// flag indicating whether the mailbox is filled or has been emptied (got_mail),
// a float indicating the battery voltage, and a float indicating the on-board
// temperature.

typedef struct __attribute__ ((packed)) sigfox_message {
  bool got_mail;
  float batt_voltage;
  float moduleTemperature;
} SigfoxMessage;

SigfoxMessage msg;

void setup() {

  if (debug == true) {
    Serial.begin(115200);
    while (!Serial) {}
  }

  if (!SigFox.begin()) {
    // Something is not right with the SigFox module.
    // We attempt to reboot.
    reboot();
  }

  //Send module to standby until we need to send a message
  SigFox.end();

  if (debug == true) {
    // Enable debug prints and LED indication if we are testing
    SigFox.debug();
  }

  // led
  pinMode(LED_PIN, OUTPUT);

  // Attach pins to switches, set internal pullup resistors and interrupt on rising
  // voltage
  pinMode(REED_MAIL, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(REED_MAIL, wake, RISING);

  pinMode(REED_CLEAR, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(REED_CLEAR, wake, RISING);

  // Visual cue that setup has completed and 5 sec delay, buying us enough time to
  // initiate new firmware upload before module goes to sleep (if needed).
  digitalWrite(LED_PIN, HIGH);
  delay(5000);
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  if (debug == true) Serial.println("Going to sleep.");

  // Sleep until an event is recognized
  LowPower.deepSleep();

  // if we get here it means that an event was received

  if (debug == true) Serial.println("Waking up.");

  // The following code is attempting to do a "hacky debounce", so that accidental
  // openings can be detected, especially of the small slit, which itself sits on
  // the front panel of the whole mailbox

  // Check first whether whole front panel of mailbox has been opened to CLEAR mail
  if(check_reed(REED_CLEAR)) {
    got_mail = false;
    sendPacket();
  }
  // Only afterwards we check if the small mailbox slit (also on the front panel!)
  // was opened
  else if(check_reed(REED_MAIL)) {
    delay(2000);
    // Make sure we didn't accidentally trigger it by opening the mailbox to empty it
    if(check_reed(REED_CLEAR)) got_mail = false;
    else got_mail = true;
    sendPacket();
  }

  // Wait 10 seconds (useful to upload code & ignore mailbox flapping)
  digitalWrite(LED_PIN, HIGH);
  delay(10000);
  digitalWrite(LED_PIN, LOW);
}

void sendPacket() {
  SigFox.begin();
  
  delay(100);

  SigFox.status();

  delay(10);

  // Fill our struct
  msg.got_mail = got_mail;
  msg.batt_voltage = getBatteryVoltage();
  msg.moduleTemperature = SigFox.internalTemperature();

  SigFox.beginPacket();
  //SigFox.write(got_mail);
  SigFox.write((uint8_t*)&msg, sizeof(SigfoxMessage));
  int ret = SigFox.endPacket();

  // shut down module, back to standby
  SigFox.end();

  if (debug == true) {
    if (ret > 0) {
      Serial.println("No transmission");
    } else {
      Serial.println("Transmission ok");
    }

    Serial.println('SIGFOX: ' + SigFox.status(SIGFOX));
    Serial.println('ATMEL: ' + SigFox.status(ATMEL));

    // Loop forever if we are testing for a single event
    //while (1) {};
  }
}

bool check_reed(int reed) {
  // Returns TRUE if given reed switch is open
  int state1 = digitalRead(reed);
  delay(20);
  int state2 = digitalRead(reed);
  if (state1 == state2 && state1 == HIGH) return true;
  return false;
}

void wake() {
  // We do nothing here.
}

void reboot() {
  NVIC_SystemReset();
  while (1);
}

float getBatteryVoltage() {
  // Get a reading of the battery voltage
  analogReadResolution(10);
  analogReference(AR_INTERNAL1V0); //AR_DEFAULT: the default analog reference of 3.3V // AR_INTERNAL1V0: a built-in 1.0V reference
 
  // read the input on analog pin 0:
  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  float voltage = sensorValue * (3.25 / 1023.0);
  return voltage;
}
