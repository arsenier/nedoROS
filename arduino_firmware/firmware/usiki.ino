void init_endcaps()
{
  pinMode(ENDCAP1, INPUT_PULLUP);
  pinMode(ENDCAP2, INPUT_PULLUP);
}

struct Endcaps
{
  bool usikleft;
  bool usikright;
};

Endcaps endcaps;

void usiki()
{
  endcaps.usikleft = !digitalRead(ENDCAP1);
  endcaps.usikright = !digitalRead(ENDCAP2);
}

bool turn_usik1()
{
  return endcaps.usikleft;
}

bool turn_usik2()
{
  return endcaps.usikright;
}