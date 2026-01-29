//servo
#include <Servo.h>
#include <Wire.h>

Servo arm, claws;


//серва
#define CLAWS_PIN 10
#define ARM_PIN 9
//
#define BUTTON_PIN 13
///
#define ENCL A5
#define ENCR A4
#define LED 8

#define SERIAL_SPEED 115200

#define K 0.35
#define V 130
#define TICKS_PER_ROTATE 484

#define OPEN 170
#define CLOSE 0

#define UP 0
#define MID 80
#define DOWN 180

#define RADIUS 23.75
#define GAUGE 235

/////////
int wl = 1023, wr = 1023, bl = 0, br = 0;
int gray = 130;

int sensL, sensR, u, err;

//motors + encoders
volatile int encL = 0, encR = 0;
uint64_t timerL = 0, timerR = 0;
int periodR = 1, periodL = 1, errL, errR, uL, uR;
float k_speed = 22, k = 0.1, speedL = 0, speedR = 0;
extern void motorRPM(int rpmL, int rpmR, int move_time = 10), move_by_dist(int dist, int vel);

int x = 0, y = 0;

void setup() {

  init_motors();
  init_servo();
  init_button();
  init_serial();
  init_encoders();
  init_led();

  
}
void loop() {
}
