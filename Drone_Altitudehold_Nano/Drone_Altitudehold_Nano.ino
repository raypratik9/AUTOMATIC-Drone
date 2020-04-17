#include <Wire.h>
#include <BM1383GLV.h>
#include<Servo.h>

BM1383GLV bm1383;
int pressure;

Servo servo_yaw, servo_roll, servo_pitch, servo_throttle;
double kp[4] = {2, 3.55, 3.55, 3.55};          //3.55
double ki[4] = {0.015, 0.005, 0.005, 0.005};       //0.003
double kd[4] = {5, 2.05, 2.05, 2.05};         //2.05
int b = 1;
float data[4] = {0, 0, 0, 0};  //Alt,Roll,Pitch,Yaw

double currentTime[4], elapsedTime[4], previousTime[4], cumError[4], rateError[4], lastError[4];
int error[4], Setpoint[4] = {2, 0, 0, 0}, out[4];

void calibration();
void pidcal(int n);

void setup() {
  byte rc;

  Serial.begin(9600);
  while (!Serial);

  Wire.begin();

  rc = bm1383.init();
}

void loop() {

  pidcal(0);
  Serial.print("out[0]=");
  Serial.println(out[0]);
  calibration();
  Altitude_hold();
}
void  Altitude_hold()
{
  if (error[0] < 5 || error[0] > -5)
  {
    servo_throttle.writeMicroseconds(1200 + out[0]);
    servo_yaw.writeMicroseconds(1500);
    servo_roll.writeMicroseconds(1500);
    servo_pitch.writeMicroseconds(1500);
  }
  else()
  {
    servo_throttle.writeMicroseconds(980);
    servo_yaw.writeMicroseconds(1500);
    servo_roll.writeMicroseconds(1500);
    servo_pitch.writeMicroseconds(1500);
  }
}
void calibration()
{
  while (b)
  {
    //delay(12000);
    for (int a = 0; a < 1000; a++)
    {
      servo_throttle.writeMicroseconds(980);
      servo_yaw.writeMicroseconds(980);
      servo_roll.writeMicroseconds(1500);
      servo_pitch.writeMicroseconds(1500);

      delay(5);
      b = 0;
    }
    Setpoint[0] = pressure - 25;
    Serial.print("Setpoint=");
    Serial.print(Setpoint[0]);
    Serial.println();
  }
  for (int a = 0; a < 5; a++)
  {
    servo_throttle.writeMicroseconds(980);
    servo_yaw.writeMicroseconds(1500);
    servo_roll.writeMicroseconds(1500);
    servo_pitch.writeMicroseconds(1500);
  }
}
void pidcal(int n)
{
  currentTime[n] = millis();                //get current time
  elapsedTime[n] = (double)(currentTime[n] - previousTime[n]);        //compute time elapsed from previous computation
  data[0] = pressure;  //Altitude in meter
  error[n] = data[n] - Setpoint[n];                              // determine error
  cumError[n] += error[n] * elapsedTime[n];                // compute integral
  rateError[n] = (error[n] - lastError[n]) / elapsedTime[n]; // compute derivative

  out[n] = kp[n] * error[n] + ki[n] * cumError[n] + kd[n] * rateError[n];          //PID output

  lastError[n] = error[n];                                //remember current error
  previousTime[n] = currentTime[n];                        //remember current time
  Serial.print("elapsedTime=");
  Serial.println(out[n]);
  if (out[n] > 1800)
    out[n] = 1800;
  if (out[n] < 1000)
    out[n] = 1000;
  //have function return the PID output
}
void get_pressure()
{
  byte rc;
  float press;

  rc = bm1383.get_val(&press);
  if (rc == 0) {
    //Serial.write("BM1383GLV (PRESS) = ");
    pressure = (press * 100) - 99000;
    //Serial.println(" [hPa]");
    Serial.print("pressure:");
    Serial.print(pressure);
    Serial.println("");
  }
}
