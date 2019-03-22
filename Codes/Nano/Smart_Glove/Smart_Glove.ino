// AUTHOR: Mavuluru Viswajith Reddy
// Arduino sketch for Sign Language Translator.
// Part of this sketch is taken from Gesture Keyboard made by Federico Terzi

#include <Wire.h>
#include <SoftwareSerial.h>

int Vin = 5;
float Vout = 0;

float R1 = 10000;
float R2 = 0;

int valC0;
int valC1;
int valC2;
int valC3;
int valC4;

int finger=0;
float buffer=0;

int flex(int);

int flex(int sensepin){
  finger =analogRead(sensepin);
  if(finger)
  {
    buffer=finger*Vin;
    Vout=(buffer)/1024.0;
    buffer=Vout/(Vin-Vout); 
    R2=R1*buffer;
  }
    return R2;
}
int Thumb, Index, Middle, Ring, Little;

boolean isBluetoothEnabled = false;

int rxPin = 2;
int txPin = 3;
SoftwareSerial bluetooth(rxPin, txPin);

int RedLedPin = 1;
int GreenLedPin = 0;
int btnPin1 = 5;

// I2C address of the MPU-6050
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int precBtn1 = HIGH;

void setup() {

  pinMode(btnPin1, INPUT_PULLUP);
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);

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

  if (precBtn1 == HIGH && resultBtn1 == LOW)
  {
    digitalWrite(RedLedPin, HIGH);
    startBatch();
  }

  if (isBluetoothEnabled)
  {
    digitalWrite(GreenLedPin, LOW);
  } else {
    digitalWrite(GreenLedPin, LOW);
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

    Thumb   = flex(A0);
    Index   = flex(A1);
    Middle  = flex(A2);
    Ring    = flex(A3);
    Little  = flex(A6);

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
    digitalWrite(RedLedPin, LOW);
    closeBatch();
  }

  precBtn1 = resultBtn1;

  if (bluetooth.available()) /* If data is available on serial port */
  {
    isBluetoothEnabled = true;
  }
  else{
    isBluetoothEnabled = false;
  }
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
