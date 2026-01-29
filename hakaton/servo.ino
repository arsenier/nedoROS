void init_servo() {
  claws.attach(CLAWS_PIN);
  arm.attach(ARM_PIN);

  open_claws();
  up();
}

void close_claws() {
  delay(100);
  for (int angle = CLOSE; angle < OPEN; angle ++) {
    claws.write(angle);
    delay(5);
  }
  delay(200);
}
void open_claws() {
  delay(100);
  for (int angle = OPEN; angle > CLOSE; angle --) {
    claws.write(angle);
    delay(5);
  }
  delay(200);
}

void down() {
  delay(100);
  arm.write(DOWN);
  delay(500);
}
void up() {
  delay(100);
  arm.write(UP);
  delay(500);
}
void mid() {
  delay(100);
  arm.write(MID);
  delay(500);
}

void check_servo() {
  down();
  delay(1000);
  mid();
  delay(1000);
  up();
  delay(1000);
  close_claws();
  open_claws();
}
