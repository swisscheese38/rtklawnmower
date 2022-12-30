#include <Arduino.h>

#define USB_BAUDRATE 9600 // Serial USB baud rate

#define PIN_LEFT_ENC 14 // Encoder Feedback
#define PIN_RIGHT_ENC 27 // Encoder Feedback
#define PIN_LEFT_DIR 13 // Direction Control
#define PIN_LEFT_PWM 26 // PWM Speed Control
#define PIN_RIGHT_PWM 25 // PWM Speed Control
#define PIN_RIGHT_DIR 12 // Direction Control

#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1

#define ENC_TICKS_PER_ROUND 456.0; // Num of ticks per revolution
#define FREQ_ENC_SAMPLING 5; // Encoder sampling frequency in Hz
#define FREQ_ENC_OUTPUT 2; // Encoder feedback output frequency in Hz

unsigned int  encSamplingPeriodMillis = 1000/FREQ_ENC_OUTPUT;
unsigned int  encOutputPeriodMillis = 1000/FREQ_ENC_OUTPUT;
float         radPerTick = TWO_PI/ENC_TICKS_PER_ROUND;

long          leftEncLastTicks;
long          rightEncLastTicks;
unsigned long leftEncLastMillis;
unsigned long rightEncLastMillis;

long          leftEncLastSampleTicks;
long          rightEncLastSampleTicks;
unsigned long leftEncLastSampleMillis;
unsigned long rightEncLastSampleMillis;

float         leftEncFreq;
float         rightEncFreq;

unsigned long lastEncOutputMillis;

float         leftSpeed = 0;
float         rightSpeed = 0;

void IRAM_ATTR onLeftTick() {
    //leftEncLastTicks += (leftSpeed >= 0) ? 1 : -1;
    //leftEncLastMillis = millis();
}

void setup() {
  Serial.begin(USB_BAUDRATE);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  ledcSetup(PWM_CHANNEL_LEFT, 500, 8);
  ledcSetup(PWM_CHANNEL_RIGHT, 500, 8);
  ledcAttachPin(PIN_LEFT_PWM, PWM_CHANNEL_LEFT);
  ledcAttachPin(PIN_RIGHT_PWM, PWM_CHANNEL_RIGHT);
  pinMode(PIN_LEFT_ENC, OUTPUT);
  pinMode(PIN_RIGHT_ENC, OUTPUT);
  ledcWrite(PWM_CHANNEL_LEFT, 0);  
  ledcWrite(PWM_CHANNEL_RIGHT, 0);  
  //attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC), onLeftTick); //[]{
    //leftEncLastTicks += (leftSpeed >= 0) ? 1 : -1;
    //leftEncLastMillis = millis();
  //}, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC), []{
    //rightEncLastTicks += (rightSpeed >= 0) ? 1 : -1;
    //rightEncLastMillis = millis();
  }, RISING);
}

void loop() {
  unsigned long currMillis = millis();
  
  // handle speed input
  if (Serial.available()) {
    String inputBuffer = Serial.readString();
    int delimPos = inputBuffer.indexOf(" ");
    float lSpeed = inputBuffer.substring(0, delimPos).toFloat();
    float rSpeed = inputBuffer.substring(delimPos).toFloat();
    if (-1 <= lSpeed && lSpeed <= 1) {
      leftSpeed = lSpeed;
      if (leftSpeed > 0) {
        digitalWrite(PIN_LEFT_DIR, LOW);
      } else {
        digitalWrite(PIN_LEFT_DIR, HIGH);
      } 
      ledcWrite(PWM_CHANNEL_LEFT, abs(leftSpeed) * 255.0);
    }
    if (-1 <= rSpeed && rSpeed <= 1) {
      rightSpeed = rSpeed;
      if (rightSpeed > 0) {
        digitalWrite(PIN_RIGHT_DIR, LOW);
      } else {
        digitalWrite(PIN_RIGHT_DIR, HIGH);
      }
      ledcWrite(PWM_CHANNEL_RIGHT, abs(rightSpeed) * 255.0);
    }
  }

  // calculate left tick frequency
  if (leftEncLastSampleMillis + encSamplingPeriodMillis < currMillis) {
    int tickDiff = leftEncLastTicks - leftEncLastSampleTicks;
    int millisDiff = leftEncLastMillis - leftEncLastSampleMillis;
    if (tickDiff == 0 || millisDiff == 0) {
      leftEncFreq = 0;
      leftEncLastSampleMillis = currMillis;
    } else {
      leftEncFreq = 1000.0 * float(tickDiff) / float(millisDiff);
      leftEncLastSampleMillis = leftEncLastMillis;
    }
    leftEncLastSampleTicks = leftEncLastTicks;
  }

  // calculate right tick frequency
  if (rightEncLastSampleMillis + encSamplingPeriodMillis < currMillis) {
    int tickDiff = rightEncLastTicks - rightEncLastSampleTicks;
    int millisDiff = rightEncLastMillis - rightEncLastSampleMillis;
    if (tickDiff == 0 || millisDiff == 0) {
      rightEncFreq = 0;
      rightEncLastSampleMillis = currMillis;
    } else {
      rightEncFreq = 1000.0 * float(tickDiff) / float(millisDiff);
      rightEncLastSampleMillis = rightEncLastMillis;
    }
    rightEncLastSampleTicks = rightEncLastTicks;
  }
  
  // output encoder feedback
  if (lastEncOutputMillis + encOutputPeriodMillis < currMillis) {
    //Serial.print(radPerTick*leftEncLastTicks);
    //Serial.print(" ");
    //Serial.print(radPerTick*leftEncFreq);
    //Serial.print(" ");
    //Serial.print(radPerTick*rightEncLastTicks);
    //Serial.print(" ");
    //Serial.println(radPerTick*rightEncFreq);
    lastEncOutputMillis = currMillis;       
  }
}