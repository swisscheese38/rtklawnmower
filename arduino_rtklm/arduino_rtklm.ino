#define USB_BAUDRATE 9600 // Serial USB baud rate

#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_RIGHT_ENC 3 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 5 // PWM Speed Control
#define PIN_RIGHT_PWM 6 // PWM Speed Control
#define PIN_RIGHT_DIR 7 // Direction Control

#define FREQ_PWM_ADJUST 100; // PWM control adjustment frequency in Hz
#define FREQ_ENC_OUTPUT 2; // Encoder feedback output frequency in Hz

unsigned int  pwmAdjustPeriodMillis = 1000/FREQ_PWM_ADJUST;
unsigned int  encOutputPeriodMillis = 1000/FREQ_ENC_OUTPUT;

long          leftEncTicks;
long          rightEncTicks;

String        inputString = "";
bool          inputStringComplete = false;

unsigned long lastPwmAdjustMillis;
unsigned long lastEncOutputMillis;

float         leftSpeedCurr;
float         leftSpeedTarget;
float         rightSpeedCurr;
float         rightSpeedTarget;

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
  leftEncTicks += (leftSpeedCurr >= 0) ? 1 : -1;
}  

void onRightEncTick() {
  rightEncTicks += (rightSpeedCurr >= 0) ? 1 : -1;
}

void loop() {
  unsigned long currMillis = millis();
  
  // handle targetspeed input
  if (inputStringComplete) {

    int delimPos = inputString.indexOf(" ");
    float leftSpeed = inputString.substring(0, delimPos).toFloat();
    float rightSpeed = inputString.substring(delimPos).toFloat();
    if (-1 <= leftSpeed && leftSpeed <= 1) {
      leftSpeedTarget = leftSpeed;
    }
    if (-1 <= rightSpeed && rightSpeed <= 1) {
      rightSpeedTarget = rightSpeed;
    }
    inputString = "";
    inputStringComplete = false;
  }
  
  // adjust pwm if not close enough to target yet
  if (lastPwmAdjustMillis + pwmAdjustPeriodMillis < currMillis) {
    if (0.004 <= abs(leftSpeedCurr - leftSpeedTarget)) {    
      leftSpeedCurr += (leftSpeedCurr < leftSpeedTarget) ? 0.004 : -0.004;
      digitalWrite(PIN_LEFT_DIR, (leftSpeedCurr > 0) ? LOW : HIGH);
      analogWrite(PIN_LEFT_PWM, abs(leftSpeedCurr) * 255.0);
    }
    if (0.004 <= abs(rightSpeedCurr - rightSpeedTarget)) {    
      rightSpeedCurr += (rightSpeedCurr < rightSpeedTarget) ? 0.004 : -0.004;
      digitalWrite(PIN_RIGHT_DIR, (rightSpeedCurr > 0) ? LOW : HIGH);
      analogWrite(PIN_RIGHT_PWM, abs(rightSpeedCurr) * 255.0);
    }
    lastPwmAdjustMillis = currMillis;
  }

  // handle encoder feedback
  if (lastEncOutputMillis + encOutputPeriodMillis < currMillis) {
    Serial.print(leftEncTicks);
    Serial.print(" ");
    Serial.println(rightEncTicks);
    lastEncOutputMillis = currMillis;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      inputStringComplete = true;
    }
  }
}