int Vin=5;
float Vout=0;
float R1=10000;    // Known Resistance
float R2=0;        // Unknown Resistance
int a2d_data=0;    
float buffer=0;            

void setup() 
{
 Serial.begin(9600);
}

void loop()
{
  a2d_data=analogRead(A6);
  if(a2d_data)
  {
    buffer=a2d_data*Vin;
    Vout=(buffer)/1024.0;
    buffer=Vout/(Vin-Vout); 
    R2=R1*buffer;
    
    Serial.print("R (ohm) = ");
    Serial.println(R2);
    
    //delay(1000);
  }
}
