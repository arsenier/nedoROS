float gx, gy, gz, ax, ay, az;

void init_gyro()
{
  Wire.begin();
  Wire.setClock(400000);
  //Serial.begin(115200);
  bool success = IMU.begin();
  if (!success) {
    while (1) {
      Serial.println("Failed to initialize IMU");
      delay(5000);
    }
  }
}

float gyro()
{
  IMU.waitForData();
  IMU.getGyro(gx, gy, gz);
  
  delay(50);

  return gz;
}
