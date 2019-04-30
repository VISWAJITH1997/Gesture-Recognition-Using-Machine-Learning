int Vin = 5;
float Vout = 0;
float R1 = 10000;  // Known Resistance
float R2 = 0;      // Unknown Resistance
int finger = 0;
float buffer = 0;

int flex(int);

int flex(int sensepin) {
  finger = analogRead(sensepin);
  if (finger)
  {
    buffer = finger * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    R2 = R1 * buffer;
    return R2;
  }
}

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int thumb = flex(A0);
  int pointer = flex(A1);
  int middle = flex(A2);
  int ring = flex(A3);
  int little = flex(A4);
  
  Serial.println(thumb);
  Serial.println(pointer);
  Serial.println(middle);
  Serial.println(ring);
  Serial.println(little);
}
