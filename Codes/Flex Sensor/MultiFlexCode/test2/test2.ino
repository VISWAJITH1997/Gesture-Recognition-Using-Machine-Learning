int Vin = 5;
float Vout = 0;
float R1 = 10000;  // Known Resistance
float R2 = 0;      // Unknown Resistance
int a2d_data = 0;
float buffer = 0;
int valC0;
int valC1;
int valC2;
int valC3;
int valC4;
int Thumb,Pointer,Middle,Ring,Little;
int sensePin = 0;

void setup() {
  // put your setup code here, to run once:
  DDRB = 255;
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //

  /// NOTE #0
  PORTB = B00000000;
  valC0 = analogRead(sensePin); // reading from photodiode
  if (valC0)
  {
    buffer = valC0 * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    Thumb = R1 * buffer;
  }

  PORTB = B00000001;
  valC1 = analogRead(sensePin); // reading from photodiode
  if (valC1)
  {
    buffer = valC1 * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    Pointer = R1 * buffer;
  }
  PORTB = B00000010;
  valC2 = analogRead(sensePin); // reading from photodiode
  if (valC2)
  {
    buffer = valC2 * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    Middle = R1 * buffer;
  }

  PORTB = B00000011;
  valC3 = analogRead(sensePin); // reading from photodiode
  if (valC3)
  {
    buffer = valC3 * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    Ring = R1 * buffer;
  }

  PORTB = B00000100;
  valC4 = analogRead(sensePin); // reading from photodiode
  if (valC4)
  {
    buffer = valC4 * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    Little = R1 * buffer;
  }


  Serial.print("Thumb = ");
  Serial.println(Thumb);

  Serial.print("Pointer = ");
  Serial.println(Pointer);

  Serial.print("Middle = ");
  Serial.println(Middle);

  Serial.print("Ring = ");
  Serial.println(Ring);

  Serial.print("Little = ");
  Serial.println(Little);

  Serial.println(" ");

  delay(1000);


}
