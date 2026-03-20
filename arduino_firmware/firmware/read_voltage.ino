float voltage()
{
  static float filtered_voltage = 12.0;
  float current_voltage = fmap(analogRead(ACUM), 0, 1023, 0.0, 12.6);

  filtered_voltage = filtered_voltage * 0.9 + current_voltage * 0.1;

  return filtered_voltage;
}
