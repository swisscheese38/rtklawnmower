#define USB_BAUDRATE 9600 // Serial USB baud rate

#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_RIGHT_ENC 3 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 9 // PWM Speed Control
#define PIN_RIGHT_PWM 10 // PWM Speed Control
#define PIN_RIGHT_DIR 7 // Direction Control

#define CONTROL_LOOP_FREQUENCY 5 // Encoder sampling frequency in Hz
#define CONTROL_LOOP_PERIOD_MS (1000/CONTROL_LOOP_FREQUENCY)

#define PWM_TICK_RATIO (250.0/1400.0) // Approximate linear ratio of tick frequency to pwm value 

String inputBuffer = "";

unsigned long lastSamplingMillis = 0;
unsigned long lastInputMillis = 0;

volatile int leftTicks = 0;
volatile unsigned long leftTicksLastMillis = 0;
int leftDir = 0;
double leftInput = 0;
double leftSetpoint = 0;
double leftOutput = 0;

volatile int rightTicks = 0;
volatile unsigned long rightTicksLastMillis = 0;
int rightDir = 0;
double rightInput = 0;
double rightSetpoint = 0;
double rightOutput = 0;

void setup() {
  Serial.begin(USB_BAUDRATE);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_LEFT_ENC, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC), onLeftEncTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC), onRightEncTick, CHANGE);
}

void onLeftEncTick() {
  leftTicks++;
  leftTicksLastMillis = millis();
}

void onRightEncTick() {
  rightTicks++;
  rightTicksLastMillis = millis();
}

void loop() {

  // handle speed input
  while (Serial.available()) {
    char inChar = (char) Serial.read();
    inputBuffer += inChar;
    if (inChar == '\n') {
      lastInputMillis = millis();
      int delimPos = inputBuffer.indexOf(" ");
      double leftVel = inputBuffer.substring(0, delimPos).toDouble();
      double rightVel = inputBuffer.substring(delimPos).toDouble();
      inputBuffer = "";

      // snap too low speeds to zero
      leftVel = (abs(leftVel) < 80) ? 0 : leftVel;
      rightVel = (abs(rightVel) < 80) ? 0 : rightVel;
      
      // only stop and don't change direction when going too fast
      if (leftDir * leftVel < 0 && abs(leftInput) > 100) {
        leftSetpoint = 0;
      } else {
        leftSetpoint = leftVel;
        leftDir = constrain(leftVel, -1, +1);
      }
      if (rightDir * rightVel < 0 && abs(rightInput) > 100) {
        rightSetpoint = 0;
      } else {
        rightSetpoint = rightVel;
        rightDir = constrain(rightVel, -1, +1);
      }
      
      // set PWM outputs to approximate values
      leftOutput = leftSetpoint * PWM_TICK_RATIO;
      rightOutput = rightSetpoint * PWM_TICK_RATIO;
    }
  }

  // blink internal LED after speed input
  digitalWrite(LED_BUILTIN, 100 > (millis() - lastInputMillis) ? HIGH : LOW);

  if (lastSamplingMillis + CONTROL_LOOP_PERIOD_MS < millis()) {
    unsigned long currSamplingMillis = millis();

    // calculate tick frequencies and reset counters
    if (leftTicks > 0) {
      leftInput = leftDir * leftTicks * (1000.0 / (leftTicksLastMillis - lastSamplingMillis));
      leftTicks = 0;
    } else {
      leftInput = 0;
    };
    if (rightTicks > 0) {
      rightInput = rightDir * rightTicks * (1000.0 / (rightTicksLastMillis - lastSamplingMillis));
      rightTicks = 0;
    } else {
      rightInput = 0;
    }
    
    // fine adjust PWMs if necessary
    double leftError = leftSetpoint - leftInput;
    if (5 < abs(leftError)) {
      leftOutput += constrain(leftError, -1, +1);
    }
    double rightError = rightSetpoint - rightInput;
    if (5 < abs(rightError)) {
      rightOutput += constrain(rightError, -1, +1);
    }

    // avoid close to zero PWMs
    if (abs(leftOutput) < 15) {
      leftOutput = (leftSetpoint == 0) ? 0 : constrain(leftOutput, -1, +1) * 15;
    }
    if (abs(rightOutput) < 15) {
      rightOutput = (rightSetpoint == 0) ? 0 : constrain(rightOutput, -1, +1) * 15;
    }
    
    // Engage new PWM values and direction
    analogWrite(PIN_LEFT_PWM, constrain(abs(leftOutput), 0, 255));
    analogWrite(PIN_RIGHT_PWM, constrain(abs(rightOutput), 0, 255));
    digitalWrite(PIN_LEFT_DIR, leftDir > 0 ? LOW : HIGH);
    digitalWrite(PIN_RIGHT_DIR, rightDir > 0 ? LOW : HIGH);

    // output encoder feedback
    Serial.print(leftInput);
    Serial.print(" ");
    Serial.print(rightInput);
    Serial.println();
    
    lastSamplingMillis = currSamplingMillis;
  }
}