
#include <Wire.h>
#include <I2C_Anything.h>

const byte MY_ADDRESS = 42;
union Buffer
{
  float data;     //first coordinate for x and next for y.
  float roll=410;
} coordinate;
//float roll = 410;
float pitch =  50;
float yaw=-16;
float Altitude=-16;
float longitude[5]={12,13,14,1,2.5};
void setup()
{

 Wire.begin (MY_ADDRESS);
 Serial.begin (9600);
Wire.onRequest(requestEvent);
}  // end of setup

void loop()
{

coordinate.roll=510;
}  // end of loop

void requestEvent (int howMany)
{
  
  I2C_writeAnything (coordinate.roll);
  I2C_writeAnything (pitch);
  I2C_writeAnything (yaw);
  I2C_writeAnything (Altitude);
  I2C_writeAnything (longitude[0]);
  I2C_writeAnything (longitude[1]);
  I2C_writeAnything (longitude[2]);
  I2C_writeAnything (longitude[3]);
  I2C_writeAnything (longitude[4]);
} 
