#define USB_BAUDRATE 9600 // Serial USB baud rate

#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_RIGHT_ENC 3 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 5 // PWM Speed Control
#define PIN_RIGHT_PWM 6 // PWM Speed Control
#define PIN_RIGHT_DIR 7 // Direction Control
#define FREQ_ENC_OUTPUT 2; // Encoder feedback output frequency in Hz

unsigned int  encOutputPeriodMillis = 1000/FREQ_ENC_OUTPUT;

long          leftEncTicks;
long          rightEncTicks;

String        inputBuffer = "";
bool          inputBufferComplete = false;

unsigned long lastEncOutputMillis;

float         leftSpeed;
float         rightSpeed;

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
  leftEncTicks += (leftSpeed >= 0) ? 1 : -1;
}  

void onRightEncTick() {
  rightEncTicks += (rightSpeed >= 0) ? 1 : -1;
}

void loop() {
  
  // handle speed input
  if (inputBufferComplete) {
    int delimPos = inputBuffer.indexOf(" ");
    float lSpeed = inputBuffer.substring(0, delimPos).toFloat();
    float rSpeed = inputBuffer.substring(delimPos).toFloat();
    if (-1 <= lSpeed && lSpeed <= 1) {
      leftSpeed = lSpeed;
      digitalWrite(PIN_LEFT_DIR, (leftSpeed > 0) ? LOW : HIGH);
      analogWrite(PIN_LEFT_PWM, abs(leftSpeed) * 255.0);
    }
    if (-1 <= rSpeed && rSpeed <= 1) {
      rightSpeed = rSpeed;
      digitalWrite(PIN_RIGHT_DIR, (rightSpeed > 0) ? LOW : HIGH);
      analogWrite(PIN_RIGHT_PWM, abs(rightSpeed) * 255.0);
    }
    inputBuffer = "";
    inputBufferComplete = false;
  }

  // handle encoder feedback
  unsigned long currMillis = millis();
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
    inputBuffer += inChar;
    if (inChar == '\n') {
      inputBufferComplete = true;
    }
  }
}