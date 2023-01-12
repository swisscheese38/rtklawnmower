#define USB_BAUDRATE 9600 // Serial USB baud rate

#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_RIGHT_ENC 3 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 9 // PWM Speed Control
#define PIN_RIGHT_PWM 10 // PWM Speed Control
#define PIN_RIGHT_DIR 7 // Direction Control

#define SAMPLING_FREQUENCY 10 // Encoder sampling frequency in Hz

#define PWM_TICK_RATIO (250.0/1400.0) // Approximate linear ratio of tick frequency to pwm value 

String inputBuffer = "";

unsigned long lastSamplingMillis = 0;
unsigned long nextSamplingMillis = 0;

volatile int leftTicks = 0;
volatile unsigned long leftTicksLastMillis = 0;
double leftSetpoint = 0;
double leftOutput = 0;

volatile int rightTicks = 0;
volatile unsigned long rightTicksLastMillis = 0;
double rightSetpoint = 0;
double rightOutput = 0;

void setup() {
  Serial.begin(USB_BAUDRATE);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_LEFT_ENC, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC, INPUT_PULLUP);
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
      int delimPos = inputBuffer.indexOf(" ");
      leftSetpoint = inputBuffer.substring(0, delimPos).toDouble();
      rightSetpoint = inputBuffer.substring(delimPos).toDouble();;
      leftOutput = leftSetpoint * PWM_TICK_RATIO;
      rightOutput = rightSetpoint * PWM_TICK_RATIO;
      inputBuffer = "";
    }
  }

  if (lastSamplingMillis + (1000/SAMPLING_FREQUENCY) < millis()) {
    unsigned long currSamplingMillis = millis();

    // calculate tick frequencies and reset counters
    double leftInput = 0;
    if (leftTicks > 0) {
      leftInput = leftTicks * (1000.0 / (leftTicksLastMillis - lastSamplingMillis));
      leftTicks = 0;
    };
    double rightInput = 0;
    if (rightTicks > 0) {
      rightInput = rightTicks * (1000.0 / (rightTicksLastMillis - lastSamplingMillis));
      rightTicks = 0;
    };
    
    // fine tune PWMs if necessary
    double leftError = leftSetpoint - leftInput;
    if (10 < abs(leftError)) {
      leftOutput += constrain(leftError, -1, +1);
    }
    double rightError = rightSetpoint - rightInput;
    if (10 < abs(rightError)) {
      rightOutput += constrain(rightError, -1, +1);
    }

    // avoid close to zero PWMs
    if (abs(leftOutput) < 15) {
      leftOutput = (leftSetpoint == 0) ? 0 : 15;
    }
    if (abs(rightOutput) < 15) {
      rightOutput = (rightSetpoint == 0) ? 0 : 15;
    }
    
    // Engage new PWM values
    analogWrite(PIN_LEFT_PWM, leftOutput);
    analogWrite(PIN_RIGHT_PWM, rightOutput);

    // output encoder feedback
    Serial.print(leftInput);
    Serial.print(" ");
    Serial.print(rightInput);
    Serial.println();
    
    lastSamplingMillis = currSamplingMillis;
  }
}