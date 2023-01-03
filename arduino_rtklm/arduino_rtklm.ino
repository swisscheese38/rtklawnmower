#define USB_BAUDRATE 9600 // Serial USB baud rate

#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_RIGHT_ENC 3 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 5 // PWM Speed Control
#define PIN_RIGHT_PWM 6 // PWM Speed Control
#define PIN_RIGHT_DIR 7 // Direction Control

#define PWM_ADJUST_FREQ_EPSILON 2 // How close the curr and target should be 
#define FREQ_ENC_SAMPLING 5 // Encoder sampling frequency in Hz
#define FREQ_ENC_OUTPUT 5 // Encoder feedback output frequency in Hz

unsigned int  encSamplingPeriodMillis = 1000/FREQ_ENC_SAMPLING;
unsigned int  encOutputPeriodMillis = 1000/FREQ_ENC_OUTPUT;

long          leftEncLastTicks;
long          rightEncLastTicks;
unsigned long leftEncLastMillis;
unsigned long rightEncLastMillis;

long          leftEncLastSampleTicks;
long          rightEncLastSampleTicks;
unsigned long leftEncLastSampleMillis;
unsigned long rightEncLastSampleMillis;

int           leftEncCurrFreq;
int           rightEncCurrFreq;
int           leftEncTargetFreq;
int           rightEncTargetFreq;

String        inputBuffer = "";
bool          inputBufferComplete = false;

unsigned long lastEncOutputMillis;

int           leftSpeed;
int           rightSpeed;

void setup() {
  Serial.begin(USB_BAUDRATE);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_LEFT_ENC, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC, INPUT_PULLUP);
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC), onLeftEncTick, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC), onRightEncTick, RISING);
}

void onLeftEncTick() {
  leftEncLastTicks += (leftSpeed >= 0) ? 1 : -1;
  leftEncLastMillis = millis();
}  

void onRightEncTick() {
  rightEncLastTicks += (rightSpeed >= 0) ? 1 : -1;
  rightEncLastMillis = millis();
}

void loop() {
  unsigned long currMillis = millis();
  
  // handle speed input
  if (inputBufferComplete) {
    int delimPos = inputBuffer.indexOf(" ");
    leftEncTargetFreq = inputBuffer.substring(0, delimPos).toFloat();
    rightEncTargetFreq = inputBuffer.substring(delimPos).toFloat();
    inputBuffer = "";
    inputBufferComplete = false;
  }

  // calculate left tick frequency and potentially adjust pwm
  if (leftEncLastSampleMillis + encSamplingPeriodMillis < currMillis) {
    int tickDiff = leftEncLastTicks - leftEncLastSampleTicks;
    int millisDiff = leftEncLastMillis - leftEncLastSampleMillis;
    if (tickDiff == 0 || millisDiff == 0) {
      leftEncCurrFreq = 0;
      leftEncLastSampleMillis = currMillis;
    } else {
      leftEncCurrFreq = 1000.0 * float(tickDiff) / float(millisDiff);
      leftEncLastSampleMillis = leftEncLastMillis;
    }
    leftEncLastSampleTicks = leftEncLastTicks;

    float leftEncFreqDiff = leftEncTargetFreq - leftEncCurrFreq;
    if (PWM_ADJUST_FREQ_EPSILON < abs(leftEncFreqDiff)) {
      leftSpeed += constrain(leftEncFreqDiff / 3.0, -20, 20);
      leftSpeed = constrain(leftSpeed, -255, 255);
      digitalWrite(PIN_LEFT_DIR, (leftSpeed > 0) ? LOW : HIGH);
      analogWrite(PIN_LEFT_PWM, abs(leftSpeed));
    }
  }

  // calculate right tick frequency and potentially adjust pwm
  if (rightEncLastSampleMillis + encSamplingPeriodMillis < currMillis) {
    int tickDiff = rightEncLastTicks - rightEncLastSampleTicks;
    int millisDiff = rightEncLastMillis - rightEncLastSampleMillis;
    if (tickDiff == 0 || millisDiff == 0) {
      rightEncCurrFreq = 0;
      rightEncLastSampleMillis = currMillis;
    } else {
      rightEncCurrFreq = 1000.0 * float(tickDiff) / float(millisDiff);
      rightEncLastSampleMillis = rightEncLastMillis;
    }
    rightEncLastSampleTicks = rightEncLastTicks;

    float rightEncFreqDiff = rightEncTargetFreq - rightEncCurrFreq;
    if (PWM_ADJUST_FREQ_EPSILON < abs(rightEncFreqDiff)) {
      rightSpeed += constrain(rightEncFreqDiff / 3.0, -20, 20);
      rightSpeed = constrain(rightSpeed, -255, 255);
      digitalWrite(PIN_RIGHT_DIR, (rightSpeed > 0) ? LOW : HIGH);
      analogWrite(PIN_RIGHT_PWM, abs(rightSpeed));
    }
  }

  // output encoder feedback
  if (lastEncOutputMillis + encOutputPeriodMillis < currMillis) {
    Serial.print(leftEncLastTicks);
    Serial.print(" ");
    Serial.print(leftEncCurrFreq);
    Serial.print(" ");
    Serial.print(rightEncLastTicks);
    Serial.print(" ");
    Serial.print(rightEncCurrFreq);
    Serial.print(" / ");
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.println(rightSpeed);
    lastEncOutputMillis = currMillis;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputBuffer += inChar;
    if (inChar == '\n') {
      inputBufferComplete = true;
    }
  }
}