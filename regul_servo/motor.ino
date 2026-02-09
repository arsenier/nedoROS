
uint64_t timerL = 0, timerR = 0;
int periodR = 1, periodL = 1;
float errL, errR, uL, uR, uIL = 0, uIR = 0;
float kMl = 80.0 / 150, kMr = 80.0 / 150;
float Amp = 5;
float tm = 250.0 / 1000;
float k_speed = Amp / kMr, k = 0.1, speedL = 0, speedR = 0, ki_speed = Amp / (tm * kMr), k_speedL = Amp / kMl, ki_speedL = Amp / (tm * kMl);

int64_t enc_get_tick_L()
{
  return encL;
}

int64_t enc_get_tick_R()
{
  return encR;
}

void motor(int motorL, int motorR) {
  digitalWrite(4, (motorL > 0));
  motorL = abs(motorL);

  if (motorL < 70) motorL = 0;
  analogWrite(5, min(255, motorL));

  digitalWrite(7, (motorR < 0));
  motorR = abs(motorR);
  if (motorR < 70) motorR = 0;
  analogWrite(6, min(255, motorR));
}

float getvel_left() {
  if (fabs(60000000 / TICKS_PER_ROTATE / periodL) < 20) return 0;
  return 60000000 / TICKS_PER_ROTATE / periodL;
}

float getvel_right() {
  if (fabs(-60000000 / TICKS_PER_ROTATE / periodR) < 20) return 0;
  return -60000000 / TICKS_PER_ROTATE / periodR;
}

void motorRPM(int rpmL, int rpmR, uint8_t move_time = Ts_ms) {
  static uint32_t timer = 0;
  uint32_t time1 = millis();
  while (millis() - timer < move_time)
    ;
  //Serial.println("dt: " + String(time1 - timer));
  timer = millis();

  speedL = 60000000 / TICKS_PER_ROTATE / periodL;
  speedR = -60000000 / TICKS_PER_ROTATE / periodR;

  if (fabs(speedL) < 20) speedL = 0;
  if (fabs(speedR) < 20) speedR = 0;

  errL = rpmL - speedL;
  errR = rpmR - speedR;
  uIR += errR * ki_speed * move_time / 1000.0;
  uIL += errL * ki_speedL * move_time / 1000.0;
  uIR = constrain(uIR, -256, 256);
  uIL = constrain(uIL, -256, 256);
  uL = errL * k_speedL + uIL;
  uR = errR * k_speed + uIR;

  /*Serial.print("encL: " + String(encL) + " encR: " + String(encR));
  Serial.print(" speedL = ");
  Serial.print(speedL);
  Serial.print(" speedR = ");
  Serial.println(speedR);*/
  /*if (rpmL == 0 && rpmR == 0)
    motor(0, 0);*/
  motor(uL, uR);
  //motor(150, 150);
  //motor(rpmL, rpmR);
}

void encoderL() {
  periodL = micros() - timerL;
  if (!digitalRead(ENCL)) {
    encL--;
    periodL = -abs(periodL);
  } else {
    encL++;
  }
  timerL = micros();
}
void encoderR() {
  periodR = micros() - timerR;
  if (!digitalRead(ENCR)) {
    encR++;
  } else {
    encR--;
    periodR = -abs(periodR);
  }
  timerR = micros();
}
void init_motors() {
  pinMode(4, 1);
  pinMode(7, 1);
}
void init_encoders() {
  attachInterrupt(1, encoderL, RISING);
  attachInterrupt(0, encoderR, RISING);
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
