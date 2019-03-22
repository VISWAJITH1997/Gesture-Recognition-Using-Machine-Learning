// AUTHOR: Federico Terzi
// Arduino sketch for the gesture controlled keyboard.
// Part of this sketch is taken from an example made by JohnChi

#include <Wire.h>
#include <SoftwareSerial.h>

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
int Thumb, Pointer, Middle, Ring, Little;
int sensePin = 0;
int flex(int, volatile uint8_t);

int flex(volatile uint8_t portb) {
  int val, finger = 0;
  PORTB = portb;
  val = analogRead(sensePin); // reading from photodiode

  if (val)
  {
    buffer = val * Vin;
    Vout = (buffer) / 1024.0;
    buffer = Vout / (Vin - Vout);
    finger = R1 * buffer;
  }
  return finger;
}

// This variable controls which communication the device will use.
// If true, the device will send data through bluetooth,
// If false, the device will send date through usb serial
boolean isBluetoothEnabled = true;

// Pins connected to the HC-06 bluetooth module
int rxPin = 2;
int txPin = 3;
SoftwareSerial bluetooth(rxPin, txPin);

// Pins used for I/O
int btnPin1 = 0;
int btnPin2 = 1;
int yellowLedPin = 6;
int redLedPin = 5;

// I2C address of the MPU-6050
const int MPU_addr = 0x68;
// Variables that will store sensor data
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Status variables, used with buttons
int precBtn1 = HIGH;
int precBtn2 = HIGH;


void setup() {

  DDRB = 255;

  // Set the pin mode of the buttons using the internal pullup resistor
  pinMode(btnPin1, INPUT_PULLUP);
  pinMode(btnPin2, INPUT_PULLUP);

  // Set the pin mode of the LEDs
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  // Start the comunication with the MPU-6050 sensor
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Start the serial communications
  Serial.begin(9600);
  bluetooth.begin(9600);
}
void loop() {
  // Read the values of the buttons
  int resultBtn1 = digitalRead(btnPin1);
  int resultBtn2 = digitalRead(btnPin2);

  // ON btn1 pressed, start the batch and light up the yellow LED
  if (precBtn1 == HIGH && resultBtn1 == LOW)
  {
    digitalWrite(yellowLedPin, HIGH);
    startBatch();
  }

  // ON btn2 pressed, toggle the communication channel ( Bluetooth/Serial )
  if (precBtn2 == HIGH && resultBtn2 == LOW)
  {
    isBluetoothEnabled = !isBluetoothEnabled;
  }

  // Controls the red LED based on the current communication channel
  if (isBluetoothEnabled)
  {
    digitalWrite(redLedPin, HIGH);
  } else {
    digitalWrite(redLedPin, LOW);
  }

  // If the btn1 is pressed, reads the data from the sensor and sends it through the communication channel
  if (resultBtn1 == LOW)
  {
    // Start the transmission with the MPU-6050 sensor
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers

    // Reads the data from the sensor
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    Thumb = flex(B00000000);
    Pointer = flex(B00000001);
    Middle = flex(B00000010);
    Ring = flex(B00000011);
    Little = flex(B00000100);
    
    // Based on the current comunication channel, it sends the data.
    if (isBluetoothEnabled)
    {
      bluetooth.print("START");
      bluetooth.print(" "); bluetooth.print(AcX);
      bluetooth.print(" "); bluetooth.print(AcY);
      bluetooth.print(" "); bluetooth.print(AcZ);
      bluetooth.print(" "); bluetooth.print(GyX);
      bluetooth.print(" "); bluetooth.print(GyY);
      bluetooth.print(" "); bluetooth.print(GyZ);
      bluetooth.print(" "); bluetooth.print(Thumb);
      bluetooth.print(" "); bluetooth.print(Pointer);
      bluetooth.print(" "); bluetooth.print(Middle);
      bluetooth.print(" "); bluetooth.print(Ring);
      bluetooth.print(" "); bluetooth.print(Little);
      bluetooth.println(" END");
    } else {
      Serial.print("START");
      Serial.print(" "); Serial.print(AcX);
      Serial.print(" "); Serial.print(AcY);
      Serial.print(" "); Serial.print(AcZ);
      Serial.print(" "); Serial.print(GyX);
      Serial.print(" "); Serial.print(GyY);
      Serial.print(" "); Serial.print(GyZ);
      Serial.print(" "); Serial.print(Thumb);
      Serial.print(" "); Serial.print(Pointer);
      Serial.print(" "); Serial.print(Middle);
      Serial.print(" "); Serial.print(Ring);
      Serial.print(" "); Serial.print(Little);
      Serial.println(" END");
    }
  }

  // Closes the batch when the button is released
  if (precBtn1 == LOW && resultBtn1 == HIGH)
  {
    digitalWrite(yellowLedPin, LOW);
    closeBatch();
  }

  // Saves the button states
  precBtn1 = resultBtn1;
  precBtn2 = resultBtn2;
}

// Sends the started batch signal
void startBatch()
{
  if (isBluetoothEnabled)
  {
    bluetooth.println("STARTING BATCH");
  } else {
    Serial.println("STARTING BATCH");
  }
}

// Sends the closed batch signal
void closeBatch()
{
  if (isBluetoothEnabled)
  {
    bluetooth.println("CLOSING BATCH");
  } else {
    Serial.println("CLOSING BATCH");
  }
}
