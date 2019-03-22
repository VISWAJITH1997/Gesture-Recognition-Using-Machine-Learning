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
int flex(int,volatile uint8_t);
int flex(volatile uint8_t portb){
  int val,finger=0;
  PORTB=portb;
   val = analogRead(sensePin); // reading from photodiode
  if (val)
  {
    buffer = val * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    if(buffer<0){
      buffer = -buffer;
    }
    finger = R1 * buffer;
  }
  return finger; 
}
void setup() {
  // put your setup code here, to run once:
  DDRB = 255;
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //

  /// NOTE #0
  Serial.print("12"); 
  Thumb=flex(B00000000);
  Pointer=flex(B00000001);
  Middle=flex(B00000010);
  Ring=flex(B00000011);
  Little=flex(B00000100);
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
