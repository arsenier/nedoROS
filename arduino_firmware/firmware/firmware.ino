//servo
#include <Servo.h>
#include <Wire.h>

Servo arm, claws;

#include <MPU9250.h>

MPU9250 IMU(Wire, -1, 0x68);

//серва
#define CLAWS_PIN 9
#define ARM_PIN 10
//
#define BUTTON_PIN 13
///
#define ENCL A0
#define ENCR A1
#define ENCLIRQ 3
#define ENCRIRQ 2
#define LED 8

#define SERIAL_SPEED 115200

#define K 0.35
#define V 130
#define TICKS_PER_ROTATE 484

#define OPEN 0.0
#define CLOSE 170.0

#define UP 10.0
#define MID 80.0
#define DOWN 180.0

#define RADIUS 23.75
#define GAUGE 235.0
#define Ts_ms 10
#define Ts_s (Ts_ms / 1000.0)

#define ENDCAP1 8
#define ENDCAP2 11

////////

//motors + encoders
volatile int64_t encL = 0, encR = 0;
extern void motorRPM(float rpmL, float rpmR, uint8_t move_time = Ts_ms), move_by_dist(int dist, int vel);

/////////////////servo//////////////////
float posarm = 0, posclaws = 0;
int time_to_claws = 500;
float t_one_it = 1.0 / (time_to_claws / float(Ts_ms));
float want_t_claws = 0.5;


void setup() {
  init_motors();
  motor(0, 0);
  init_servo();
  init_button();
  init_serial();
  init_encoders();
  motor(100, 100);
  delay(50);
  motor(0, 0);
  init_led();
  init_gyro();
  init_endcaps();

  Serial.println(get_x());
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  readrpi();
  odom();
  usiki();

  motorRPM(velL_from_rpi(), velR_from_rpi());
  //motorRPM(2, -2);
  static float t = 0;

  // vel_est_tick();

  Serial.print(" getvel_left = ");
  Serial.print(getvel_left());
  Serial.print(" getvel_right = ");
  Serial.println(getvel_right());

  // Serial.print(" theta = ");
  // Serial.print(get_theta());
  // Serial.print(" x = ");
  // Serial.print(get_x());
  // Serial.print(" gyro = ");
  // Serial.print(gyro());
  // Serial.print(" y = ");
  // Serial.println(get_y());

  // Serial.print(" encl = ");
  // Serial.print(int(encL));
  // Serial.print(" encr = ");
  // Serial.print(int(encR));

  // Serial.print(" enclAngle = ");
  // Serial.print(getLangle());
  // Serial.print(" encrAngle = ");
  // Serial.println(getRangle());

  posarm = constrain(fmap(t, (1 - want_t_claws), 1, DOWN, UP), UP, DOWN);
  posclaws = constrain(fmap(t, 0, want_t_claws, OPEN, CLOSE), OPEN, CLOSE);
  claws.write(posclaws);
  arm.write(posarm);
  if (gripper_form_rpi())
    t += t_one_it;
  else
    t -= t_one_it;

  t = constrain(t, 0, 1);
}
