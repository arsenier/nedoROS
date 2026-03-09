struct Wegives {
  float left_motor_speed;
  float right_motor_speed;
  bool gripper;
  uint8_t checksum;
};

struct Wesend {
  float x;
  float y;
  float theta;
  bool usik1;
  bool usik2;
  uint8_t check_sum;
};

union Union_wegives {
  uint8_t buf[10];
  Wegives wegives_time;
};

union Union_wesend {
  uint8_t buf[15];
  Wesend wesend_time;
};

Wegives wegives;

void readrpi() {
  Union_wegives union_wegives;
  Union_wesend union_wesend;
  union_wesend.wesend_time.x = get_x();
  union_wesend.wesend_time.y = get_y();
  union_wesend.wesend_time.theta = get_theta();
  union_wesend.wesend_time.usik1 = turn_usik1();
  union_wesend.wesend_time.usik2 = turn_usik2();

  while (Serial.available()) {
    uint8_t check = 0;
    if (Serial.read() == 0x01) {
      Serial.readBytes(union_wegives.buf, 10);
      for (size_t i = 0; i < 10; i++)
        check ^= union_wegives.buf[i];

      if (check == 0) {
        wegives = union_wegives.wegives_time;

        Serial.write(0x01);
        uint8_t check_out = 0;
        for (size_t i = 0; i < 14; i++)
          check_out ^= union_wesend.buf[i];
        union_wesend.wesend_time.check_sum = check_out;
        Serial.write(union_wesend.buf, sizeof(union_wesend.buf));
      }
    }
  }
}

float velL_from_rpi() {
  return wegives.left_motor_speed;
}

float velR_from_rpi() {
  return wegives.right_motor_speed;
}

bool gripper_form_rpi() {
  return wegives.gripper;
}