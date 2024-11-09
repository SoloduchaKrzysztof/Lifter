//
// A sketch for the programmable power supply based on Arduino Mega 2560.
// Voltage outputs with feedbacks. 
//

#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <math.h>

// Global constants
const int analogA0 = A0; // Analog pin A0 used for reading the output voltage from the DAC driver
const int analogA4 = A4; // Analog pin A4 used for reading the output voltage from the PWM driver
const int pwmPin = 3; // Define the PWM pin as pin 3
const float max_v1 = 10.0;  // Maximum voltage for channel 1
const float min_v1 = -10.0; // Minimum voltage for channel 1
const float max_v2 = 4.5;   // Maximum voltage for channel 2; Minimum voltage = 0.0 V
const float op_amp_gain = 2.385488184;  // Gain of OPA549 for DAC output [0-5] V
const float A0_coeff = 0.311; // Transfer coefficient (mean value) for reading the DAC output

const int tau = 200;  // Characteristic time (ms) of the RC filter at the PWM output with C = 100 uF and R = 2k
const float bs_DAC =  2.0 * (4.98 / 1023.0) * A0_coeff;  // bit sensitivity for DAC output
const float bs_PWM =  2.0 * (4.98 / 255.0);  // bit sensitivity for PWM output 

// Array of the A0 readings used to calculate the average value
const int numSamples = 50;
long int samples[numSamples]; // Array of the integers to hold the samples from A0

// The A0 transfer coefficient (y) vs. the AO digital reading (x) averaged over 300 samples (float)
const int numPoints = 27;
const float x[] = {2.2, 12.8, 28.3, 44, 59.8, 75.6, 91.3, 107.3, 122.6, 138.1, 153.5, 185.2, 217, 248.4, 279.4, 311, 342.3, 373, 405, 436.4, 467.4, 499, 531, 562.3, 594, 625, 639};
const float y[] = {0.129172324, 0.254330002, 0.283467893, 0.297491039, 0.301666844, 0.304655182, 0.306729892, 0.308711688, 0.309233738, 0.309947284, 0.310833362, 0.312174556, 0.312996633, 0.31375712, 0.313538274, 0.314164545, 0.314401594, 0.314420282, 0.314944769, 0.315054213, 0.315359619, 0.315473969, 0.316005707, 0.315866165, 0.316023268, 0.315942055, 0.316125456};

// Variables
String chan1, chan2;
int v_raw, dutyCycle;
int DAC_value;
float v1, v2, v1_out, v2_out;

Adafruit_MCP4725 dac; // Declare DAC object variable

// Variables for some previous values
String previous_pin12_13 = "HIGH";  // Relays are OFF

bool flag_send = 1, 
     sign = 0, //1 if '-' and 0 if '+'
     previous_sign = 0;  // Polarity of the DAC output
     
void setup() { 

  Serial.begin(19200); // Initialize serial communication
  delay(100);

  // Initialize the pins as outputs
  pinMode(2, OUTPUT);  // LED "+" for the DAC output
  pinMode(4, OUTPUT);  // LED "+" for the DAC output 
  pinMode(5, OUTPUT);  // LED "+" for the PWM output
  pinMode(12, OUTPUT);  // Relay 1; H-bridge
  pinMode(13, OUTPUT);  // Relay 2; H-bridge
  pinMode(52, OUTPUT);  // Enable (LOW) or Disable (HIGH) OPA549
  pinMode(pwmPin, OUTPUT); // PWM for pwmPin = 3
  
  // Initialize DAC and the DAC output
  dac.begin(0x62); // I2C address (0x62) of DAC
  dac.setVoltage(0, false);  // DAC is OFF
  digitalWrite(52, HIGH);  // OPA549 OFF
  digitalWrite(2, LOW);  // LED HIGH 
  digitalWrite(4, LOW);  // LED OFF

  // Initialize PWM output 
  pinMode(pwmPin, OUTPUT); // pwmPin = 3
  analogWrite(pwmPin, 0);  // PWM is OFF
  digitalWrite(5, LOW);  // LED OFF
 
  // Relays controlling the polarity on the DAC output
  digitalWrite(12, HIGH);  // Relay 1 is OFF; activated by LOW
  digitalWrite(13, HIGH);  // Relay 2 is OFF; activated by LOW 
 
}

// Function definition
float DAC_Vout(int DAC_value){
  dac.setVoltage(DAC_value, false);  // Set the DAC output
  delay(10);

  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    int reading = analogRead(A0); // Read the analog input on pin A0
    samples[i] = reading;
    sum = sum + reading;
    delay(10); // Small delay to give some time between readings
  }

  float meanValue = (float)sum / numSamples;

  float sigma = 0.0;
  for (int i = 0; i < numSamples; i++) {
    sigma = sigma + pow(meanValue - (float)samples[i], 2);
  }

  sigma = pow(sigma / (numSamples - 1), 0.5);

  long newsum = 0.0;
  int m = 0;
  for (int i = 0; i < numSamples; i++) {
    float zscore = abs((float)samples[i] - meanValue) / sigma; 
    if(zscore < 3.0) {
      m = m + 1;
      newsum = newsum + samples[i];
    }
  }

  if (m != 0) {
    meanValue = (float)newsum / m;
  }

  if (meanValue < 2.2) {
    meanValue = 2.2;
  } else if (meanValue > 639) {
    meanValue = 639;
  }
  
  float coeff = linearInterpolation(meanValue);
  float analogVoltage = (meanValue * 4.98) / (1023.0 * coeff);
  return analogVoltage;
}

// Function definition
float PWM_Vout(int dutyCycle) {
  // Change the duty cycle by specifying a value between 0 (off) and 255 (full on)
  analogWrite(pwmPin, dutyCycle);  // Set the duty cycle
  delay(tau * 6.0);  // Delay time to allow the full capacitor charge

  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    int reading = analogRead(A4); // Read the analog input on pin A4
    samples[i] = reading;
    sum = sum + reading;
    delay(10); // Small delay to give some time between readings
  }

  float meanValue = (float)sum / numSamples;

  float sigma = 0.0;
  for (int i = 0; i < numSamples; i++) {
    sigma = sigma + pow(meanValue - (float)samples[i], 2);
  }

  sigma = pow(sigma / (numSamples - 1), 0.5);

  long newsum = 0.0;
  int m = 0;
  for (int i = 0; i < numSamples; i++) {
    float zscore = abs((float)samples[i] - meanValue) / sigma; 
    if(zscore < 3.0) {
      m = m + 1;
      newsum = newsum + samples[i];
    }
  }

  if (m != 0) {
    meanValue = (float)newsum / m;
  }

  return (meanValue * 4.98) / 1023.0;

}

// Function definition
float linearInterpolation(float x_val) {
  // Find the correct interval for x_val
  int interval = -1;
  for (int i = 0; i < numPoints - 1; i++) {
    if (x_val >= x[i] && x_val <= x[i + 1]) {
      interval = i;
      break;
    }
  }

  if (interval == -1) {
    Serial.println("Error: x_val is out of bounds.");
    return 0.0;
  }

  // Perform linear interpolation
  float x0 = x[interval];
  float x1 = x[interval + 1];
  float y0 = y[interval];
  float y1 = y[interval + 1];

  float y_val = y0 + ((x_val - x0) * (y1 - y0)) / (x1 - x0);

  return y_val;
}

String receivedData = "";
void ParseSerial() {
  char space = '0';
  String temp_string = "";
  for (int i = 0; i < receivedData.length(); i++) {
      if (receivedData[i] == ' ')
      {
        space++;
        switch (space) {
          case '1': {chan1 = temp_string; temp_string = ""; break; }
          case '2': {v1 = temp_string.toFloat();  temp_string = ""; break; }
          case '3': {chan2 = temp_string; temp_string = ""; break; }
        }
      }
    else if (i == receivedData.length() - 1)
        v2 = temp_string.toFloat();
    else
        temp_string += receivedData[i];
  }
}

void loop() {
  // Check if data is available to read
  while (receivedData[receivedData.length() - 1] != '\n') {
    if (Serial.available()){
      receivedData += (char) Serial.read();
      if (receivedData[receivedData.length() - 1] == '\n')
      {
        receivedData.toLowerCase();
        ParseSerial();
      }
   }
  } 

  // DAC output
  if (chan1 == "on") {
    sign = signbit(v1); //1 if '-' and 0 if '+'   
    if (sign != previous_sign) {
      digitalWrite(52, HIGH); // Disable OPA459
      previous_sign = sign;
      if (previous_pin12_13 == "HIGH") {
        digitalWrite(12, LOW);
        digitalWrite(13, LOW);
        previous_pin12_13 = "LOW"; 
      } else {
        digitalWrite(12, HIGH);
        digitalWrite(13, HIGH);
        previous_pin12_13 = "HIGH";         
      }
    }

    // Polarity indication
    if (sign == 1) {
      digitalWrite(2, LOW);  
      digitalWrite(4, HIGH);
    } else if (sign == 0) {
      digitalWrite(2, HIGH);  
      digitalWrite(4, LOW);
    }
    
    DAC_value = round(((fabs(v1) / op_amp_gain) / 4.98) * 4095);  // Convert the DAC voltage to the DAC value (0-4095)
    digitalWrite(52, LOW); // Enable OPA459
    v1_out = DAC_Vout(DAC_value);  // Output voltage how it is seen by the A0 analog input
    while ((fabs(v1_out - fabs(v1)) > bs_DAC) ) {
        if (v1_out > fabs(v1) and DAC_value > 0)
          DAC_value--;
        else if (v1_out < fabs(v1) and DAC_value < 4095)
          DAC_value++;
        v1_out = DAC_Vout(DAC_value);
    }
  } 

// PWM output
  if (chan2 == "on") {
    digitalWrite(5, HIGH);  // LED ON
    dutyCycle = (v2 / 4.98) * 255;  // Convert the PWM voltage to the duty cycle value (0-255)
    v2_out = PWM_Vout(dutyCycle);  // Output voltage how it is seen by the A4 analog input
    
    while ((fabs(v2_out - v2) > bs_PWM) ){
        if (v2_out > v2 and dutyCycle > 0)
          dutyCycle--;
        else if (v2_out < v2 and dutyCycle < 255)
          dutyCycle++;
        v2_out = PWM_Vout(dutyCycle);
    }
  } 
  if (flag_send){ 
    Serial.println( ((sign) ? ("-") : ("")) + String(v1_out) + " " + String(v2_out));
    flag_send = 0;
  }
  delay(100);
}

