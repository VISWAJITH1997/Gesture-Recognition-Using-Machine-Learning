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
int valC5;
int valC6;
int valC7;
int valC8;
int valC9;
int valC10;
int valC11;
int valC12;
int valC13;
int valC14;
int valC15;

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
    R2 = R1 * buffer;
  }

  PORTB = B00000001;
  valC1 = analogRead(sensePin); // reading from photodiode

  PORTB = B00000010;
  valC2 = analogRead(sensePin); // reading from photodiode

  PORTB = B00000011;
  valC3 = analogRead(sensePin); // reading from photodiode

  PORTB = B00000100;
  valC4 = analogRead(sensePin); // reading from photodiode

  PORTB = B00000101;
  valC5 = analogRead(sensePin); // reading from photodiode

  PORTB = B00000110;
  valC6 = analogRead(sensePin); // reading from photodiode

  PORTB = B00000111;
  valC7 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001000;
  valC8 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001001;
  valC9 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001010;
  valC10 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001011;
  valC11 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001100;
  valC12 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001101;
  valC13 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001110;
  valC14 = analogRead(sensePin); // reading from photodiode

  PORTB = B00001111;
  valC15 = analogRead(sensePin); // reading from photodiode

  Serial.print("R (ohm) = ");
  Serial.println(R2);

  Serial.print("Channel1 = ");
  Serial.println(valC1);

  Serial.print("Channel2 = ");
  Serial.println(valC2);

  Serial.print("Channel3 = ");
  Serial.println(valC3);

  Serial.print("Channel4 = ");
  Serial.println(valC4);

  Serial.print("Channel5 = ");
  Serial.println(valC5);

  Serial.print("Channel6 = ");
  Serial.println(valC6);

  Serial.print("Channel7 = ");
  Serial.println(valC7);

  Serial.print("Channel8 = ");
  Serial.println(valC8);

  Serial.print("Channel9 = ");
  Serial.println(valC9);

  Serial.print("Channel10 = ");
  Serial.println(valC10);

  Serial.print("Channel11 = ");
  Serial.println(valC11);


  Serial.print("Channel12 = ");
  Serial.println(valC12);

  Serial.print("Channel13 = ");
  Serial.println(valC13);

  Serial.print("Channel14 = ");
  Serial.println(valC14);


  Serial.print("Channel15 = ");
  Serial.println(valC15);

  Serial.println(" ");

  delay(1000);


}
