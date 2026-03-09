void init_button() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void move_by_dist(int dist, int vel) {
  static int current_dist;
  dist = dist / (2 * PI * RADIUS) * TICKS_PER_ROTATE;
  encL = 0;
  encR = 0;
  while (abs(encL + encR) / 2 < abs(dist)) {
    current_dist = encL / TICKS_PER_ROTATE * 2 * PI * RADIUS;
    motorRPM(vel, vel);
  }
  motor(0, 0);
}
void turn(int angle, int vel) {
  int dist = angle * GAUGE / RADIUS / 2;
  encL = 0;
  while (abs(encL) < abs(dist)) {
    motorRPM(abs(vel) * sign(angle), -abs(vel) * sign(angle));
  }
  stop_motor();
}

int sign(int in) {
  return in / abs(in);
}

void move_arc_right(int radius, int angle, int time) {
  float turnoverL = (((radius - 175) * 2 * PI * angle / 360) / (2 * PI * RADIUS));
  float turnoverR =  (((radius + 175) * 2 * PI * angle / 360) / (2 * PI * RADIUS));
  // Serial.println(turnoverL);
  // Serial.println(turnoverR);
  float velL = turnoverL / time;
  velL = 255 * velL / 2;
  // Serial.println(velL);
  float velR = turnoverR / time;
  velR = 255 * velR / 2;
  // Serial.println(velR);
  encL = 0;
  while (encR < ((turnoverR * TICKS_PER_ROTATE)/2.45)) {
    motor(int(velL), int(velR));
  }
  stop_motor();
}
