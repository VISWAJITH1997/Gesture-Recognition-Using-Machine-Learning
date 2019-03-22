int Vin=5;
float Vout=0;
float R1=10000;    // Known Resistance
float R2=0;        // Unknown Resistance
int finger=0;    
float buffer=0;            

int flex(int);

int flex(int sensepin){
  finger=analogRead(A0);
  if(finger)
  {
    buffer=finger*Vin;
    Vout=(buffer)/1024.0;
    buffer=Vout/(Vin-Vout); 
    R2=R1*buffer;
    return R2;
  }
}

void setup() 
{
 Serial.begin(9600);
}

void loop()
{
  int test = flex(A6);
  Serial.println(test);
}
