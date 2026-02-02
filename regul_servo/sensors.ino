void init_serial() {
  Serial.begin(SERIAL_SPEED);
}

void wait_button() {
  while (digitalRead(BUTTON_PIN))
    ;
}

void init_led() {
  pinMode(LED, 1);
}
void light(int a) {
  for (int i = 0; i < a; i++) {
    digitalWrite(LED, 1);
    delay(250);
    digitalWrite(LED, 0);
    delay(250);
  }
}
