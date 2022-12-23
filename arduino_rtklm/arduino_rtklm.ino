#define USB_BAUDRATE 9600 // Serial USB baud rate
#define PIN_LEFT_ENC 2 // Encoder Feedback
#define PIN_LEFT_DIR 4 // Direction Control
#define PIN_LEFT_PWM 5 // PWM Speed Control
#define FREQ_ENC_SAMPLING 5; // Encoder feedback sampling frequency in Hz

unsigned int  encSamplingPeriodMillis = 1000/FREQ_ENC_SAMPLING;

unsigned long lastSampledLeftEncTicks;
unsigned long lastSampledEncMillis;
float         lastSampledLeftEncFreq;

unsigned long currLeftEncTicks;
String        inputString = "";
bool          inputStringComplete = false;

void setup() {
  Serial.begin(USB_BAUDRATE);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_LEFT_ENC, INPUT_PULLUP);
  analogWrite(PIN_LEFT_PWM, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC), []{currLeftEncTicks++;}, RISING);
  Serial.println("Control Motor speed and direction by Serial monitor");
  Serial.println("Send values -10 through 10");
}

void loop() {
  if (inputStringComplete) {
    int motorspeed = inputString.toInt();
    if (-10 <= motorspeed && motorspeed <= 10) {
      digitalWrite(PIN_LEFT_DIR, motorspeed > 0 ? LOW : HIGH);
      analogWrite(PIN_LEFT_PWM, map(abs(motorspeed), 0, 10, 0, 255));
    }
    inputString = "";
    inputStringComplete = false;
  }
  unsigned long currMillis = millis();
  if (lastSampledEncMillis + encSamplingPeriodMillis < currMillis) {
    lastSampledLeftEncFreq = 1000.0 * float(currLeftEncTicks-lastSampledLeftEncTicks) / float(currMillis-lastSampledEncMillis);
    Serial.println(lastSampledLeftEncFreq);
    lastSampledLeftEncTicks = currLeftEncTicks;
    lastSampledEncMillis = currMillis;
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