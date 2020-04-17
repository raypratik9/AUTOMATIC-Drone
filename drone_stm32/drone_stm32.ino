#include<Servo.h>
#include "MPU9250.h"

Servo servo_yaw, servo_roll, servo_pitch, servo_throttle;
int b = 1, c = 1;

float data[4]={0,0,0,0};
//volatile float rollangle, data[1];
volatile float roll, pitch, yaw;
volatile float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ, pressure;
#define A 0.962
#define dt 0.020

double kp[4] = {3.55, 3.55, 3.55, 3.55};          //3.55
double ki[4] = {0.005, 0.005, 0.005, 0.005};       //0.003
double kd[4] = {2.05, 2.05, 2.05, 2.05};         //2.05
double currentTime[4],elapsedTime[4],previousTime[4],cumError[4],rateError[4],lastError[4];
int error[4],Setpoint[4]={0,0,0,0},out[4];
void pidcal(int n);

MPU9250 IMU(Wire, 0x68);
int status;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servo_roll.attach(5);
  servo_pitch.attach(6);

  servo_throttle.attach(7);
  servo_yaw.attach(8);
 // status = IMU.begin();
 // Serial.println(status);
//  if (status < 0) {
//    Serial.println("IMU initialization unsuccessful");
//    Serial.println("Check IMU wiring or try cycling power");
//    Serial.print("Status: ");
//    Serial.println(status);
//    while (1) {}
//  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("1");
  while (b)
  {
    delay(10000);
    for (int a = 0; a < 1000; a++)
    {
      servo_throttle.writeMicroseconds(980);
      servo_yaw.writeMicroseconds(980);
      servo_roll.writeMicroseconds(1500);
      servo_pitch.writeMicroseconds(1500);
      delay(10);
      b = 0;
    }
  }
  for(int a=0;a<50;a++)
  {
    servo_throttle.writeMicroseconds(980);
  servo_yaw.writeMicroseconds(1500);
   servo_roll.writeMicroseconds(1500);
   servo_pitch.writeMicroseconds(1500);
  }
  while (c)
  {
    servo_throttle.writeMicroseconds(1000);
    servo_yaw.writeMicroseconds(1500);
    servo_roll.writeMicroseconds(1500);
    servo_pitch.writeMicroseconds(1500);
    for (int a = 0; a < 60; a++)
    {
      servo_throttle.writeMicroseconds(1300);
      servo_yaw.writeMicroseconds(1500);
      servo_roll.writeMicroseconds(1500-out[1]);
      servo_pitch.writeMicroseconds(1500+out[0]);
      getdata();
  pidcal(0);
  pidcal(1);
    }
    c = 0;
  }
  
}
void getdata()
{
  IMU.readSensor();
  accelX = IMU.getAccelX_mss();
  accelY = IMU.getAccelY_mss();
  accelZ = IMU.getAccelZ_mss();
  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();
  magX = IMU.getMagX_uT();
  magY = IMU.getMagY_uT();
  magZ = IMU.getMagZ_uT();
//  pressure = bmp.readPressure();
  r_p_y();
}
void r_p_y()
{
  data[0] = atan2(accelY, accelZ) * 180 / PI; // FORMULA FOUND ON INTERNET
  data[1] = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI; //FORMULA FOUND ON INTERNET
  if (data[0] > 0 && data[0] < 180)
    data[0] = map(data[0], 0, 180, 180, 0);
  if (data[0] < 0 && data[0] > -180)
    data[0] = map(data[0], 0, -180, -180, 0);
  data[0] = data[0] * (-1);
  data[1] = data[1] * (-1);

  //Using Complemetary Filter
  roll = A * (roll + gyroX * dt) + (1 - A) * data[0];
  pitch = A * (pitch + gyroY * dt) + (1 - A) * data[1];

  yaw = magX;
  //   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
  //   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));
  Serial.print(data[0]);
  Serial.print("\t");
  Serial.print(data[1]);
  Serial.print("\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.println();
//  Serial.print(F("Pressure = "));
//  //Serial.print(bmp.readPressure());
//  Serial.println(" Pa");
  Serial.print(out[0]+1500);
  Serial.print("\t");
  Serial.print(1500-out[1]);
  Serial.print("\t");
//  Serial.print(yaw);
//  Serial.print("\t");
  Serial.println();

}
void pidcal(int n)
{
//  currentTime[n] = millis();                //get current time
//  elapsedTime[n] = (double)(currentTime[n] - previousTime[n]);        //compute time elapsed from previous computation

  error[n] = Setpoint[n] - data[n];                                // determine error
//  cumError[n] += error[n] * elapsedTime[n];                // compute integral
//  rateError[n] = (error[n] - lastError[n]) / elapsedTime[n]; // compute derivative

  out[n] = kp[n] * error[n];  // + ki[n] * cumError[n] + kd[n] * rateError[n];          //PID output
//
//  lastError[n] = error[n];                                //remember current error
//  previousTime[n] = currentTime[n];                        //remember current time

                                       //have function return the PID output
}
