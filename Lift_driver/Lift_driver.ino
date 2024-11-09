// The sketch for the lifting platform project based on Arduino Mega 2560
// Krzysztof Soloducha and Dmitriy Makhnovskiy
// Plymouth, UK, July 2024.

#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;  // DAC shield for Arduino

// Initial parameters.
const float V = 5.0;  // DAC output voltage (V) for the motor
const float refArduino = 4.98;  // Arduino reference voltage (slighly less than 5 V)
const float opampGAIN = 2.385488184;  // OPA549 gain (calibrated)
const int DACVALUE = ((V / opampGAIN) / refArduino) * 4095;  // Integer level for DAC to provide the 5 V output
const int delayTIME = 200;  // Delay time after switching the relays to change the polatiry of the H-bridge

// GPIO configuration.
const int move_upBUTTON = 23;
const int move_downBUTTON = 25;
const int e_stopBUTTON = 27;

const int topSwitch1 = 29;
const int topSwitch2 = 31;
const int bottomSwitch1 = 33;
const int bottomSwitch2 = 35;

const int DACLED1 = 2;  // Polarity LED
const int DACLED2 = 4;  // Polarity LED
const int pwmLED = 5;  // PWM LED - it will be always OFF
const int move_upLED = 37;  // LED on the control box
const int move_downLED = 39;  // LED on the control box
const int e_stopLED = 41;  // LED on the control box
const int relay1 = 12;  // Controlling the relay 1
const int relay2 = 13;  // Controlling the relay 2
const int enOPA = 52;  // Controlling the OPA549 op amp (ONN/OFF)
int relaySTATE;  // State of the relay output pins (HIGH/LOW)

void setup() {
  // Declarion of GPIO pins: INPUT/OUTPUT.
  pinMode(move_upBUTTON, INPUT);
  pinMode(move_downBUTTON, INPUT);
  pinMode(e_stopBUTTON, INPUT);

  pinMode(topSwitch1, INPUT);
  pinMode(topSwitch2, INPUT);
  pinMode(bottomSwitch1, INPUT);
  pinMode(bottomSwitch2, INPUT);

  pinMode(DACLED1, OUTPUT);
  pinMode(DACLED2, OUTPUT);
  pinMode(pwmLED, OUTPUT);
  pinMode(move_upLED, OUTPUT);
  pinMode(move_downLED, OUTPUT);
  pinMode(e_stopLED, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(enOPA, OUTPUT);

  dac.begin(0x62); // The I2C address for the DAC

  // Initial configuration of the driver.
  dac.setVoltage(0, false);
  digitalWrite(enOPA, HIGH); // OPA549 OFF (activated by LOW)
  delay(delayTIME);
  digitalWrite(DACLED1, LOW);
  digitalWrite(DACLED2, LOW);
  digitalWrite(pwmLED, LOW);
  digitalWrite(move_upLED, LOW);
  digitalWrite(move_downLED, LOW);
  digitalWrite(e_stopLED, LOW);
  digitalWrite(relay1, HIGH);  // OFF
  digitalWrite(relay2, HIGH);  // OFF
  relaySTATE = HIGH;
  dac.setVoltage(0, false);
}

void loop() {
  digitalWrite(enOPA, HIGH); // OPA549 OFF
  dac.setVoltage(0, false);
  delay(delayTIME);
  dac.setVoltage(DACVALUE, false);

  // E-STOP
  while (digitalRead(e_stopBUTTON) == HIGH) {
    digitalWrite(enOPA, HIGH); // OPA549 OFF
    digitalWrite(e_stopLED, HIGH);
    digitalWrite(DACLED1, LOW);
    digitalWrite(DACLED2, LOW);
    digitalWrite(move_upLED, LOW);
    digitalWrite(move_downLED, LOW);
  }

  // No motion.
  while (digitalRead(move_upBUTTON) == LOW && digitalRead(move_downBUTTON) == LOW && digitalRead(e_stopBUTTON) == LOW) {
    digitalWrite(DACLED1, LOW);
    digitalWrite(DACLED2, LOW);
    digitalWrite(move_upLED, LOW);
    digitalWrite(move_downLED, LOW);
    digitalWrite(e_stopLED, LOW);
    digitalWrite(enOPA, HIGH); // OPA549 OFF
  }

  // If both the UP and DOWN buttons on the control box were pressed at the same time by mistake. 
  while (digitalRead(move_upBUTTON) == HIGH && digitalRead(move_downBUTTON) == HIGH && digitalRead(e_stopBUTTON)) {
    digitalWrite(DACLED1, LOW);
    digitalWrite(DACLED2, LOW);
    digitalWrite(move_upLED, LOW);
    digitalWrite(move_downLED, LOW);
    digitalWrite(e_stopLED, LOW);
    digitalWrite(enOPA, HIGH); // OPA549 OFF
  }

  // Moving UP.
  while (digitalRead(move_upBUTTON) == HIGH && digitalRead(topSwitch1) == LOW && digitalRead(topSwitch2) == LOW && digitalRead(e_stopBUTTON) == LOW) {
    digitalWrite(DACLED1, HIGH);
    digitalWrite(DACLED2, LOW);
    digitalWrite(move_upLED, HIGH);
    digitalWrite(move_downLED, LOW);
    digitalWrite(e_stopLED, LOW);
    // Changing the polarity if required.
    // The polarity cannot be changed under the output voltage. Therefore, it is reset first.
    if (relaySTATE == LOW) {
      digitalWrite(enOPA, HIGH); // OPA549 OFF
      digitalWrite(relay1, HIGH);  // OFF
      digitalWrite(relay2, HIGH);  // OFF
      relaySTATE = HIGH;
    }
    delay(delayTIME);  // Some delay time is required to avoid instability
    digitalWrite(enOPA, LOW); // OPA549 ON
  }
  
  // Moving DOWN.
  while (digitalRead(move_upBUTTON) == LOW && digitalRead(move_downBUTTON) == HIGH && digitalRead(bottomSwitch1) == LOW && digitalRead(bottomSwitch2) == LOW && digitalRead(e_stopBUTTON) == LOW) {
    digitalWrite(DACLED1, LOW);
    digitalWrite(DACLED2, HIGH);
    digitalWrite(move_upLED, LOW);
    digitalWrite(move_downLED, HIGH);
    digitalWrite(e_stopLED, LOW);
    // Changing the polarity if required.
    // The polarity cannot be changed under the output voltage. Therefore, it is reset first.
    if (relaySTATE == HIGH) {
      digitalWrite(enOPA, HIGH); // OPA549 OFF
      digitalWrite(relay1, LOW);  // ON
      digitalWrite(relay2, LOW);  // ON
      relaySTATE = LOW;
    }
    delay(delayTIME);  // Some delay time is required to avoid instability
    digitalWrite(enOPA, LOW); // OPA549 ON
  }
p
  // If the platform has reached the maximum height and the top limit switches are activated.
  while (digitalRead(move_downBUTTON) == LOW && (digitalRead(topSwitch1) == HIGH || digitalRead(topSwitch2) == HIGH)) {
    digitalWrite(enOPA, HIGH); // OPA549 OFF
    digitalWrite(DACLED1, LOW);
    digitalWrite(DACLED2, LOW);
    digitalWrite(move_downLED, LOW);
    digitalWrite(move_upLED, millis() % 500 < 250 ? HIGH : LOW);  // Flashing UP LED
    digitalWrite(e_stopLED, LOW);
  }

  // If the platform has descended to the minimum height and the limit switches are activated."
  while (digitalRead(move_upBUTTON) == LOW && (digitalRead(bottomSwitch1) == HIGH || digitalRead(bottomSwitch2) == HIGH)) {
    digitalWrite(enOPA, HIGH); // OPA549 OFF
    digitalWrite(DACLED1, LOW);
    digitalWrite(DACLED2, LOW);
    digitalWrite(move_downLED, millis() % 500 < 250 ? HIGH : LOW);  // Flashing DOWN LED
    digitalWrite(move_upLED, LOW);
    digitalWrite(e_stopLED, LOW);
  }
}