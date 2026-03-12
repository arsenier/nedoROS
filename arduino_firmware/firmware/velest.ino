static float oldAngleL = 0, oldAngleR = 0;

float velL = 0, velR = 0;

void vel_est_tick()
{
  velL = (getLangle() - oldAngleL) / Ts_s;
  velR = (getRangle() - oldAngleR) / Ts_s;

  oldAngleL = getLangle(), oldAngleR = getRangle();
}

float getvel_left() {
  return velL;
}

float getvel_right() {
  return velR;
}
