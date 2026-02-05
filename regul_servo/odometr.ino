
//////////////////odom
float x = 0, y = 0, theta = 0;

float dist_i = 0.0, theta_i = 0.0;

void odom()
{
  float velL = getvel_left() * RADIUS;
  float velR = getvel_right() * RADIUS;
  float vel = (velL + velR) / 2;

  theta_i = gyro() / GAUGE;

  x += vel * cos(theta) * Ts_s;
  y += vel * sin(theta) * Ts_s;
  theta += theta_i * Ts_s;
}

float get_theta()
{
  return theta;
}

float get_x()
{
  return x;
}

float get_y()
{
  return y;
}