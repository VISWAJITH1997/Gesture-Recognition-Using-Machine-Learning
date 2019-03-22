// AUTHOR: Mavuluru Viswajith Reddy
// Arduino sketch for Sign Language Translator.
// Part of this sketch is taken from Gesture Keyboard made by Federico Terzi

#include <Wire.h>
#include <SoftwareSerial.h>

int Vin = 5;
float Vout = 0;

float R1 = 10000;
float R2 = 0;

int a2d_data = 0;
float buffer = 0;

int valC0;
int valC1;
int valC2;
int valC3;
int valC4;

int Thumb, Index, Middle, Ring, Little;
int sensePin = 0;
int flex(int, volatile uint8_t);

int flex(volatile uint8_t portb) {
  int val, finger = 0;
  PORTB = portb;
  val = analogRead(sensePin);

  if (val)
  {
    buffer = val * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    finger = R1 * buffer;
  }
  return finger;
}

boolean isBluetoothEnabled = true;

int rxPin = 2;
int txPin = 3;
SoftwareSerial bluetooth(rxPin, txPin);

int yellowLedPin = 0;
int redLedPin = 1;
int btnPin1 = 5;
int btnPin2 = 6;

// I2C address of the MPU-6050
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int precBtn1 = HIGH;
int precBtn2 = HIGH;

void setup() {

  DDRB = 255;
  pinMode(btnPin1, INPUT_PULLUP);
  pinMode(btnPin2, INPUT_PULLUP);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(9600);
  bluetooth.begin(9600);
}
void loop() {
  int resultBtn1 = digitalRead(btnPin1);
  int resultBtn2 = digitalRead(btnPin2);

  if (precBtn1 == HIGH && resultBtn1 == LOW)
  {
    digitalWrite(yellowLedPin, HIGH);
    startBatch();
  }

  if (precBtn2 == HIGH && resultBtn2 == LOW)
  {
    isBluetoothEnabled = !isBluetoothEnabled;
  }

  if (isBluetoothEnabled)
  {
    digitalWrite(redLedPin, HIGH);
  } else {
    digitalWrite(redLedPin, LOW);
  }

  if (resultBtn1 == LOW)
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);

    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_X_H) & 0x3C (ACCEL_X_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_Y_H) & 0x3E (ACCEL_Y_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_Z_H) & 0x40 (ACCEL_Z_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_H) & 0x42 (TEMP_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_X_H) & 0x44 (GYRO_X_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_Y_H) & 0x46 (GYRO_Y_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_Z_H) & 0x48 (GYRO_Z_L)

    Thumb   = flex(B00000000);
    Index   = flex(B00000001);
    Middle  = flex(B00000010);
    Ring    = flex(B00000011);
    Little  = flex(B00000100);

    bluetooth.print("START");
    bluetooth.print(" "); bluetooth.print(AcX);
    bluetooth.print(" "); bluetooth.print(AcY);
    bluetooth.print(" "); bluetooth.print(AcZ);
    bluetooth.print(" "); bluetooth.print(GyX);
    bluetooth.print(" "); bluetooth.print(GyY);
    bluetooth.print(" "); bluetooth.print(GyZ);
    bluetooth.print(" "); bluetooth.print(Thumb);
    bluetooth.print(" "); bluetooth.print(Index);
    bluetooth.print(" "); bluetooth.print(Middle);
    bluetooth.print(" "); bluetooth.print(Ring);
    bluetooth.print(" "); bluetooth.print(Little);
    bluetooth.println(" END");

    Serial.print("START");
    Serial.print(" "); Serial.print(AcX);
    Serial.print(" "); Serial.print(AcY);
    Serial.print(" "); Serial.print(AcZ);
    Serial.print(" "); Serial.print(GyX);
    Serial.print(" "); Serial.print(GyY);
    Serial.print(" "); Serial.print(GyZ);
    Serial.print(" "); Serial.print(Thumb);
    Serial.print(" "); Serial.print(Index);
    Serial.print(" "); Serial.print(Middle);
    Serial.print(" "); Serial.print(Ring);
    Serial.print(" "); Serial.print(Little);
    Serial.println(" END");
  }

  if (precBtn1 == LOW && resultBtn1 == HIGH)
  {
    digitalWrite(yellowLedPin, LOW);
    closeBatch();
  }

  precBtn1 = resultBtn1;
  precBtn2 = resultBtn2;
}

void startBatch()
{
  bluetooth.println("STARTING BATCH");
  Serial.println("STARTING BATCH");
}

void closeBatch()
{
  bluetooth.println("CLOSING BATCH");
  Serial.println("CLOSING BATCH");
}
