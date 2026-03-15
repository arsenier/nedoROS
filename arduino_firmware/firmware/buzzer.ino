void init_buz()
{
  pinMode(A3, 1);
}

void voice()
{
  digitalWrite(A3, 1);
  delay(500);
  digitalWrite(A3, 0);
  delay(500);
}