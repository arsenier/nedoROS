void motor(int motorL, int motorR) {
  digitalWrite(7, (motorL < 0));
  motorL = abs(motorL);
  analogWrite(6, min(255, motorL));

  digitalWrite(4, (motorR < 0));
  motorR = abs(motorR);
  analogWrite(5, min(255, motorR));
}
void motorRPM(int rpmL, int rpmR, int move_time=10) {
  static uint32_t timer;
  timer = millis();
  do {
    speedL = 60000000 / TICKS_PER_ROTATE / periodL;
    speedR = 60000000 / TICKS_PER_ROTATE / periodR;

    errL = rpmL - speedL;
    errR = rpmR - speedR;
    uL = errL * k_speed;
    uR = errR * k_speed;
    motor(uL, uR);
  } while (millis() - timer < move_time);
}

void encoderL() {
  periodL = micros() - timerL;
  if (!digitalRead(ENCL)) {
    encL --;
    periodL = -abs(periodL);
  } else {
    encL ++;
  }
  timerL = micros();
}
void encoderR() {
  periodR = micros() - timerR;
  if (!digitalRead(ENCR)) {
    encR ++;
  } else {
    encR --;
    periodR = -abs(periodR);
  }
  timerR = micros();
}
void init_motors() {
  pinMode(4, 1);
  pinMode(7, 1);
}
void init_encoders() {
  attachInterrupt(0, encoderL, RISING);
  attachInterrupt(1, encoderR, RISING);
}

void stop_motor() {
  k_speed = 10;
  motorRPM(0, 0, 200);
  motor(0, 0);
  delay(200);
  k_speed = 22;
}

void check_motors() {
  motor(-128, 0);
  delay(1000);
  motor(256, 0);
  delay(1000);
  motor(0, -128);
  delay(1000);
  motor(0, 256);
  delay(1000);
  motor(-128, 128);
  delay(1000);
  motor(128, -128);
  delay(1000);
  motor(0, 0);
}
