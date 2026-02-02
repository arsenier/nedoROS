#define BUTTON_X 11
#define BUTTON_Y 12
void skiki_x() {
  static bool last_state = 0;
  bool current_state = digitalRead(BUTTON_X);
  if (current_state == 1 && last_state == 0) {
    delay(5);
    x ++;
  }
  last_state = current_state;
}
void skiki_y() {
  static bool last_state = 0;
  bool current_state = digitalRead(BUTTON_Y);
  if (current_state == 1 && last_state == 0) {
    delay(5);
    y ++;
  }
  last_state = current_state;
}
void init_buttons() {
  pinMode(BUTTON_X, INPUT_PULLUP);
  pinMode(BUTTON_Y, INPUT_PULLUP);
}
