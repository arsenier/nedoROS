
//////////////////odom
float x = 0.0, y = 0, theta = 0;

float dist_i = 0.0, theta_i = 0.0;

void odom() {
  static int64_t old_encL = 0;
  static int64_t old_encR = 0;

  float velL = (enc_get_tick_L() - old_encL) / Ts_s;
  float velR = -(enc_get_tick_R() - old_encR) / Ts_s;

  old_encL = enc_get_tick_L();
  old_encR = enc_get_tick_R();

  // float velL = getvel_left() * RADIUS;
  // float velR = getvel_right() * RADIUS;
  float vel = (velL + velR) / 2;

  if (fabs(gyro()) > 0.04)
    theta_i = gyro();
  else
    theta_i = 0;

  float k = 0.2/500;

  x += k*vel * cos(theta) * Ts_s;
  y += k*vel * sin(theta) * Ts_s;
  theta += theta_i * Ts_s;

  
  Serial.print(" velL = ");
  Serial.print(velL);
  Serial.print("\tvelR = ");
  Serial.print(velR);
  Serial.print("\tvel = ");
  Serial.print(vel);
}

float get_theta() {
  return theta;
}

float get_x() {
  return x;
}

float get_y() {
  return y;
}