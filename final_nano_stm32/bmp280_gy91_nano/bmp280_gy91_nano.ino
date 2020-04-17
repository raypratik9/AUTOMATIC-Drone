#include <I2C_Anything.h>

#include "MPU9250.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (9)
#define A 0.962
#define dt 0.020
const byte MY_ADDRESS = 42;

void r_p_y();
void get_raw_data();

volatile float rollangle, pitchangle;
volatile float roll, pitch, yaw;
volatile float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ, pressure;
volatile float Altitude;
//Adafruit_BMP280 bmp; // I2C
Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
MPU9250 IMU(SPI, 10);
int status;

void setup() {
  // put your setup code here, to run once:
  // serial to display data
  Serial.begin(9600);
//  
//     Serial.println(F("BMP280 test"));
//
//  if (!bmp.begin()) {
//    //    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
//    while (1);
//  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  while (!Serial);

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    //    Serial.println("IMU initialization unsuccessful");
    //    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  Wire.begin (MY_ADDRESS);
  Wire.onRequest(requestEvent);
}
void requestEvent (int howMany)
{

  I2C_writeAnything (rollangle);
  I2C_writeAnything (pitchangle);
  I2C_writeAnything (yaw);
  I2C_writeAnything (pressure);
}
void r_p_y()
{
  rollangle = atan2(accelY, accelZ) * 180 / PI; // FORMULA FOUND ON INTERNET
  pitchangle = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI; //FORMULA FOUND ON INTERNET
  yaw = 180 * atan (accelZ/sqrt(accelX*accelX + accelZ*accelZ))/PI;;
  if (rollangle > 0 && rollangle < 180)
    rollangle = map(rollangle, 0, 180, 180, 0);
  if (rollangle < 0 && rollangle > -180)
    rollangle = map(rollangle, 0, -180, -180, 0);
  rollangle = rollangle * (-1);
  pitchangle = pitchangle * (-1);

  //Using Complemetary Filter
  //roll = A * (roll + gyroX * dt) + (1 - A) * rollangle;
 // pitch = A * (pitch + gyroY * dt) + (1 - A) * pitchangle;

  
  //   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
  //   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));
}
void get_raw_data()
{
  accelX = IMU.getAccelX_mss();
  accelY = IMU.getAccelY_mss();
  accelZ = IMU.getAccelZ_mss();
  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();
  magX = IMU.getMagX_uT();
  magY = IMU.getMagY_uT();
  magZ = IMU.getMagZ_uT();
  pressure = bmp.readPressure();
  Altitude = bmp.readAltitude(1013.25);
}
void loop() {
  // put your main code here, to run repeatedly:
  IMU.readSensor();
  get_raw_data();
  r_p_y();
  // display the data
  //  Serial.print(accelX);
  //  Serial.print("\t");
  //  Serial.print(accelY);
  //  Serial.print("\t");
  //  Serial.print(accelZ);
  //  Serial.print("\t");
  //  Serial.print(gyroX);
  //  Serial.print("\t");
  //  Serial.print(gyroY);
  //  Serial.print("\t");
  //  Serial.print(gyroZ);
  //  Serial.print("\t");
  //  Serial.print(magX);
  //  Serial.print("\t");
  //  Serial.print(magY);
  //  Serial.print("\t");
  //  Serial.print(magZ);
  //  Serial.print("\t");
  //  Serial.println();
  Serial.print(rollangle);
  Serial.print("\t");
  Serial.print(pitchangle);
  Serial.print("\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.println();
  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  
}
