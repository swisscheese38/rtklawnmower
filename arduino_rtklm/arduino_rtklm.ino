#define USB_BAUDRATE 9600 // Serial USB baud rate

#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_RIGHT_ENC 3 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 9 // PWM Speed Control
#define PIN_RIGHT_PWM 10 // PWM Speed Control
#define PIN_RIGHT_DIR 7 // Direction Control

#define SAMPLING_FREQUENCY 5 // Encoder sampling frequency in Hz

const unsigned int elapsedTime = (1000.0 / SAMPLING_FREQUENCY);
const double kp = 0.1;
const double ki = 0.001;
const double kd = 1;

String inputBuffer = "";

unsigned long nextSamplingMillis = millis();

volatile int leftTicks = 0;
double leftSetpoint = 0;
double leftLastError = 0;
double leftCumError = 0;

volatile int rightTicks = 0;
double rightSetpoint = 0;
double rightLastError = 0;
double rightCumError = 0;

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
  leftTicks++;
}

void onRightEncTick() {
  rightTicks++;
}

void loop() {

  // handle speed input
  while (Serial.available()) {
    char inChar = (char) Serial.read();
    inputBuffer += inChar;
    if (inChar == '\n') {
      int delimPos = inputBuffer.indexOf(" ");
      leftSetpoint = inputBuffer.substring(0, delimPos).toDouble();
      rightSetpoint = inputBuffer.substring(delimPos).toDouble();
      inputBuffer = "";
    }
  }
  
  if (nextSamplingMillis < millis()) {

    // calculate tick frequencies and reset tick counters
    double leftInput = leftTicks * SAMPLING_FREQUENCY;
    leftTicks = 0;
    double rightInput = rightTicks * SAMPLING_FREQUENCY;
    rightTicks = 0;

    // compute left PID and adjust PWM
    double leftError = leftSetpoint - leftInput; // P
    leftCumError += leftError * elapsedTime; // I
    double leftRateError = (leftError - leftLastError) / elapsedTime; // D
    double leftOutput = (kp * leftError) + (ki * leftCumError) + (kd * leftRateError);
    leftLastError = leftError;
    analogWrite(PIN_LEFT_PWM, constrain(abs(leftOutput), 0, 255));
    digitalWrite(PIN_LEFT_DIR, (leftOutput > 0) ? LOW : HIGH);
    
    // compute right PID and adjust PWM
    double rightError = rightSetpoint - rightInput; // P
    rightCumError += rightError * elapsedTime; // I
    double rightRateError = (rightError - rightLastError) / elapsedTime; // D
    double rightOutput = (kp * rightError) + (ki * rightCumError) + (kd * rightRateError);
    rightLastError = rightError;
    analogWrite(PIN_RIGHT_PWM, constrain(abs(rightOutput), 0, 255));
    digitalWrite(PIN_RIGHT_DIR, (rightOutput > 0) ? LOW : HIGH);
    
    // output encoder feedback
    Serial.print(leftInput);
    Serial.print(" ");
    Serial.print(rightInput);
    Serial.println();
    
    nextSamplingMillis += elapsedTime;
  }
}