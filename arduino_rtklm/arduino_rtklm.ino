#define USB_BAUDRATE 9600 // Serial USB baud rate
#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 5 // PWM Speed Control
#define FREQ_ENC_SAMPLING 10; // Encoder feedback sampling frequency in Hz
#define FREQ_PWM_ADJUST 100; // PWM control adjustment frequency in Hz

unsigned int  encSamplingPeriodMillis = 1000/FREQ_ENC_SAMPLING;
unsigned int  pwmAdjustPeriodMillis = 1000/FREQ_PWM_ADJUST;

unsigned long lastSampledLeftEncTicks;
unsigned long lastSampledLeftEncMillis;

unsigned long lastLeftEncTicks;
unsigned long lastLeftEncMillis;

float         leftEncFreq;

String        inputString = "";
bool          inputStringComplete = false;

unsigned long lastPwmAdjustMillis;
float         motorspeedCurr;
float         motorspeedTarget;

void setup() {
  Serial.begin(USB_BAUDRATE);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_LEFT_ENC, INPUT_PULLUP);
  analogWrite(PIN_LEFT_PWM, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC), onLeftEncTick, RISING);
  Serial.println("Control Motor speed and direction by Serial monitor");
  Serial.println("Send values -10 through 10");
}

void onLeftEncTick() {
  lastLeftEncTicks++;
  lastLeftEncMillis = millis();
}

void loop() {
  unsigned long currMillis = millis();
  
  // handle targetspeed input
  if (inputStringComplete) {
    float motorspeed = inputString.toFloat();
    if (-10 <= motorspeed && motorspeed <= 10) {
      motorspeedTarget = motorspeed / 10.0;
    }
    inputString = "";
    inputStringComplete = false;
  }
  
  // adjust pwm if not close enough to target yet
  if (lastPwmAdjustMillis + pwmAdjustPeriodMillis < currMillis) {
    if (0.004 <= abs(motorspeedCurr - motorspeedTarget)) {    
      motorspeedCurr += motorspeedCurr < motorspeedTarget ? 0.004 : -0.004;
      digitalWrite(PIN_LEFT_DIR, motorspeedCurr > 0 ? LOW : HIGH);
      analogWrite(PIN_LEFT_PWM, abs(motorspeedCurr) * 255.0);
    }
    lastPwmAdjustMillis = currMillis;
  }

  // handle encoder feedback
  if (lastSampledLeftEncMillis + encSamplingPeriodMillis < currMillis) {
    unsigned int tickDiff = lastLeftEncTicks - lastSampledLeftEncTicks;
    unsigned int millisDiff = lastLeftEncMillis - lastSampledLeftEncMillis;
    if (tickDiff == 0) {
      leftEncFreq = 0;
      lastSampledLeftEncMillis = currMillis;
    } else {
      leftEncFreq = 1000.0 * float(tickDiff) / float(millisDiff);
      lastSampledLeftEncMillis = lastLeftEncMillis;
    }
    lastSampledLeftEncTicks = lastLeftEncTicks;
    Serial.println(leftEncFreq);
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