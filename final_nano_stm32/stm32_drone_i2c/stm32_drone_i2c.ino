
#include <Wire.h>
#include <I2C_Anything.h>
//#include<Servo.h>
//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>

//Servo servo_yaw, servo_roll, servo_pitch, servo_throttle;
const byte SLAVE_ADDRESS = 42;
long data[4];     //first coordinate for x and next for y.
int b = 1;
int pv_data = 0;
static const int RXPin = 4, TXPin = 3;
//static const uint32_t GPSBaud = 9600;
//TinyGPSPlus gps;
//SoftwareSerial ss(RXPin, TXPin);

float pid_p[4];
float pid_i[4];
float pid_d[4];

double kp[4] = {3.55, 3.55, 3.55, 3.55};          //3.55
double ki[4] = {0.005, 0.005, 0.005, 0.005};       //0.003
double kd[4] = {2.05, 2.05, 2.05, 2.05};         //2.05

double currentTime[4],elapsedTime[4],previousTime[4],cumError[4],rateError[4],lastError[4];
int error[4],Setpoint[4],inp[4],out[4];
void pidcal_roll();
void pidcal_pich();
void pidcal_yaw();
void pidcal_altitude();
void altitude_Hold();
void callibration();

void setup()
{
  pinMode(PC13,OUTPUT);
  digitalWrite(PC13,LOW);
  Serial.begin(115200);
  Serial.print("0");
  Wire.begin ();
  Serial.print("1");
  
//  servo_yaw.attach(PA8);
//  servo_throttle.attach(PA9);
//  servo_roll.attach(PA10);
//  servo_pitch.attach(PB6);
  Serial.print("2");
//  Serial.println(F("DeviceExample.ino"));
//  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
//  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
//  Serial.println(F("by Mikal Hart"));
  Serial.println();
}  // end of setup


void loop()
{
 Serial.print("hi");
  Wire.requestFrom(42, 16); 
 
  while (Wire.available())
  {
    Serial.print("   @   ");
    I2C_readAnything (data[0]);
    I2C_readAnything (data[1]);
    I2C_readAnything (data[2]);
    I2C_readAnything (data[3]);
//    if (data[0] > (pv_data + 45) && data[0] < (pv_data - 45))
//      data[0] = 0;
    Serial.print(data[0]);
    Serial.print("      ");
    Serial.print(data[1]);
    Serial.print("      ");
    Serial.print(data[2]);
    Serial.print("      ");
    Serial.print(data[3]);
    Serial.println();
    pv_data = data[0];
  }
   Serial.print(data[0]);
    Serial.print("      ");
    Serial.print(data[1]);
    Serial.print("      ");
    Serial.print(data[2]);
    Serial.print("      ");
    Serial.print(data[3]);
    Serial.println();
  getdata();
  callibration();
  pidcal_altitude();
  pidcal_roll();
  pidcal_pitch();
  pidcal_yaw();
  altitude_Hold();
}  // end of loop

//void calibration()
//{
//  while (b)
//  {
//    for (int a = 0; a < 1000; a++)
//    {
//      servo_throttle.writeMicroseconds(980);
//      servo_yaw.writeMicroseconds(980);
//      servo_roll.writeMicroseconds(1500);
//      servo_pitch.writeMicroseconds(1500);
//      delay(30);
//      b = 0;
//    }
//  }
//  for (int a = 0; a < 50; a++)
//  {
//    servo_throttle.writeMicroseconds(980);
//    servo_yaw.writeMicroseconds(1500);
//    servo_roll.writeMicroseconds(1500);
//    servo_pitch.writeMicroseconds(1500);
//  }
//}
void getdata()
{

     // request 2 bytes from slave device #8
//  while (ss.available() > 0)
//    if (gps.encode(ss.read()))
//      displayInfo();
//
//  if (millis() > 5000 && gps.charsProcessed() < 10)
//  {
//    Serial.println(F("No GPS detected: check wiring."));
//    while (true);
//  }
  
}
//void displayInfo()
//{
//  Serial.print(F("Location: "));
//  if (gps.location.isValid())
//  {
//    Serial.print(gps.location.lat(), 6);
//    Serial.print(F(","));
//    Serial.print(gps.location.lng(), 6);
//  }
//  else
//  {
//    Serial.print(F("INVALID"));
//  }
//  Serial.println("altitude              ");
//  Serial.print(gps.altitude.feet());
//  Serial.println();
//}
void altitude_Hold()
{

}
void pidcal_yaw()
{
  currentTime[3] = millis();                //get current time
  elapsedTime[3] = (double)(currentTime[3] - previousTime[3]);        //compute time elapsed from previous computation

  error[3] = Setpoint[3] - inp[3];                                // determine error
  cumError[3] += error[3] * elapsedTime[3];                // compute integral
  rateError[3] = (error[3] - lastError[3]) / elapsedTime[3]; // compute derivative

  out[3] = kp[3] * error[3] + ki[3] * cumError[3] + kd[3] * rateError[3];          //PID output

  lastError[3] = error[3];                                //remember current error
  previousTime[3] = currentTime[3];                        //remember current time

                                       //have function return the PID output
}
void pidcal_pitch()
{
  currentTime[2] = millis();                //get current time
  elapsedTime[2] = (double)(currentTime[2] - previousTime[2]);        //compute time elapsed from previous computation

  error[2] = Setpoint[2] - inp[2];                                // determine error
  cumError[2] += error[2] * elapsedTime[2];                // compute integral
  rateError[2] = (error[2] - lastError[2]) / elapsedTime[2]; // compute derivative

  out[2] = kp[2] * error[2] + ki[2] * cumError[2] + kd[2] * rateError[2];          //PID output

  lastError[2] = error[2];                                //remember current error
  previousTime[2] = currentTime[2];                        //remember current time

}
void pidcal_roll()
{
  currentTime[1] = millis();                //get current time
  elapsedTime[1] = (double)(currentTime[1] - previousTime[1]);        //compute time elapsed from previous computation

  error[1] = Setpoint[1] - inp[1];                                // determine error
  cumError[1] += error[1] * elapsedTime[1];                // compute integral
  rateError[1] = (error[1] - lastError[1]) / elapsedTime[1]; // compute derivative

   out[1] = kp[1] * error[1] + ki[1] * cumError[1] + kd[1] * rateError[1];          //PID output

  lastError[1] = error[1];                                //remember current error
  previousTime[1] = currentTime[1];                        //remember current time

}
void pidcal_altitude()
{
  currentTime[4] = millis();                //get current time
  elapsedTime[4] = (double)(currentTime[4] - previousTime[4]);        //compute time elapsed from previous computation

  error[4] = Setpoint[4] - inp[4];                                // determine error
  cumError[4] += error[4] * elapsedTime[4];                // compute integral
  rateError[4] = (error[4] - lastError[4]) / elapsedTime[4]; // compute derivative

  out[4] = kp[4] * error[4] + ki[4] * cumError[4] + kd[4] * rateError[4];          //PID output

  lastError[4] = error[4];                                //remember current error
  previousTime[4] = currentTime[4];                        //remember current time

}
