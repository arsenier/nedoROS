
uint64_t timerL = 0, timerR = 0;
int periodR = 1, periodL = 1;
float errL, errR, uL, uR, uIL = 0, uIR = 0;
float kMl = 4.5 / 150, kMr = 4.4 / 150;
float Amp = 4;
float tm = 250.0 / 1000;
float k_speed = Amp / kMr, k = 0.1, ki_speed = Amp / (tm * kMr), k_speedL = Amp / kMl, ki_speedL = Amp / (tm * kMl);
float kEncLtoangle = 2 * PI / 1650.0;
float kEncRtoangle = 2 * PI / 1000.0;

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

void motorRPM(float rpmL, float rpmR, uint8_t move_time = Ts_ms) {
  static uint32_t timer = 0;
  uint32_t time1 = millis();

  while (millis() - timer < move_time)
    ;
  //Serial.println("dt: " + String(time1 - timer));
  timer = millis();

  vel_est_tick();

  errL = rpmL - getvel_left();
  errR = rpmR - getvel_right();
  uIL += errL * ki_speedL * move_time / 1000.0;
  uIR += errR * ki_speed * move_time / 1000.0;
  uIL = constrain(uIL, -256, 256);
  uIR = constrain(uIR, -256, 256);

  // if(fabs(errL) < 0.1) uIL *= 0.98;
  // if(fabs(errR) < 0.1) uIR *= 0.98;

  uL = errL * k_speedL + uIL;
  uR = errR * k_speed + uIR;

  // log("errL", errL);
  // log("errR", errR);
  // log("uIL", uIL);
  // log("uIR", uIR);
  // log("uL", uL);
  // log("uR", uR);

  // // Serial.print("encL: " + String(encL) + " encR: " + String(encR));
  // Serial.print(" speedL = ");
  // Serial.print(speedL);
  // Serial.print(" speedR = ");
  // Serial.println(speedR);
  /*if (rpmL == 0 && rpmR == 0)
    motor(0, 0);*/
  motor(uL, uR);
  //motor(150, 150);
  //motor(rpmL, rpmR);
}

void encoderL() {
  periodL = micros() - timerL;
  if (!digitalRead(ENCL) != !digitalRead(ENCLIRQ)) {
    encL--;
    periodL = -abs(periodL);
  } else {
    encL++;
  }
  timerL = micros();
}
void encoderR() {
  periodR = micros() - timerR;
  if (!digitalRead(ENCR) == !digitalRead(ENCRIRQ)) {
    encR++;
  } else {
    encR--;
    periodR = -abs(periodR);
  }
  timerR = micros();
}
float getLangle()
{
  return encL * kEncLtoangle;
}

float getRangle()
{
  return encR * kEncRtoangle;
}
void init_motors() {
  pinMode(4, 1);
  pinMode(7, 1);
}
void init_encoders() {
  attachInterrupt(1, encoderL, CHANGE);
  attachInterrupt(0, encoderR, CHANGE);
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
