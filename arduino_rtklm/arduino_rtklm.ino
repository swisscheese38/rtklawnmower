#define pinTickLeft 2
#define pinReverseLeft 4
#define pinPwmLeft 5

long tick = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pinPwmLeft, OUTPUT);
  pinMode(pinReverseLeft, OUTPUT);
  pinMode(pinTickLeft, INPUT_PULLUP);
  analogWrite(pinPwmLeft, 0);
  attachInterrupt(digitalPinToInterrupt(pinTickLeft), tickLeft, CHANGE);
  Serial.println("Control Motor speed by Serial monitor");
  Serial.println("Send -1, 0 or 1 to control Motor direction");
}

void tickLeft() {
  tick = tick + 1;
}

void loop() {
  if (Serial.available() > 0) {
    int motorspeed = Serial.parseInt();
    if (motorspeed == 0) {
      analogWrite(pinPwmLeft, 0);
      Serial.println("stop");
    }
    if (motorspeed == 1) {
      analogWrite(pinPwmLeft, 20);
      digitalWrite(pinReverseLeft, LOW);
      Serial.println("low");
    }
    if (motorspeed == -1) {
      analogWrite(pinPwmLeft, 20);
      digitalWrite(pinReverseLeft, HIGH);
      Serial.println("high");
    }


    Serial.print("You send: ");
    Serial.print(motorspeed);
    Serial.println();

    //int pwm = map(motorspeed, 0, 9, 0, 255);
    
    //Serial.print("Converted to PWM: ");
    //Serial.print(pwm);
    //Serial.println();

    //analogWrite(pinPwmLeft, pwm);

    Serial.println(tick);
  }
}
