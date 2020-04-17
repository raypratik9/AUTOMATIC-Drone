
#include <Wire.h>
#include <I2C_Anything.h>

const byte SLAVE_ADDRESS = 42;
float ticks[9];     //first coordinate for x and next for y.
void setup()
{
  Wire.begin ();
  Serial.begin(9600);

}  // end of setup


void loop()
{
  Wire.requestFrom(42, 36);    // request 2 bytes from slave device #8

  while (Wire.available())
  { // slave may send less than requested
    for (int i = 0; i < 9; i++)
      I2C_readAnything (ticks[i]);// receive a byte as character
    Serial.print(ticks[0]);
    Serial.print("      ");
    Serial.print(ticks[1]);      // print the character
    Serial.print("      ");
    Serial.print(ticks[2]);
    Serial.print("      ");
    Serial.print(ticks[3]);
    Serial.print("      ");
    Serial.print(ticks[4]);      // print the character
    Serial.print("      ");
    Serial.print(ticks[5]);
    Serial.print("      ");
    Serial.print(ticks[6]);
    Serial.print("      ");
    Serial.print(ticks[7]);
    Serial.print("      ");
    Serial.print(ticks[8]);
    Serial.println();
  }
}  // end of loop
