float voltage()
{
  return fmap(analogRead(ACUM), 0, 1023, 0.0, 12.6);
}