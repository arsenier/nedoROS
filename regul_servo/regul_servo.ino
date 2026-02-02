//servo
#include <Servo.h>
#include <Wire.h>

Servo arm, claws;


//серва
#define CLAWS_PIN 9
#define ARM_PIN 10
//
#define BUTTON_PIN 13
///
#define ENCL A4
#define ENCR A5
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

////////

//motors + encoders
volatile int encL = 0, encR = 0;
extern void motorRPM(int rpmL, int rpmR, int move_time = 10), move_by_dist(int dist, int vel);

/////////////////servo//////////////////
float posarm = 0, posclaws = 0;
int time_to_claws = 3000;
float t_one_it = 1.0 / (time_to_claws / 10.0);
float want_t_claws = 0.5;

int x = 0, y = 0;

void setup() {

  init_motors();
  init_servo();
  init_button();
  init_serial();
  init_encoders();
  init_led();

}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  motorRPM(20, 20);
  static float t = 0;

  Serial.println(posarm);
  Serial.println(t);
  posarm = constrain(fmap(t, (1 - want_t_claws), 1, DOWN, UP), UP, DOWN);
  posclaws = constrain(fmap(t, 0, want_t_claws, OPEN, CLOSE), OPEN, CLOSE);
  claws.write(posclaws);
  arm.write(posarm);

   t = sin(millis() / 1000.0);
  //t += t_one_it;
  //t = constrain(t, 0, 1);
}
