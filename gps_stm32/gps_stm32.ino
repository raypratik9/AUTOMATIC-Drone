              
#include <TinyGPS++.h>         //Library for using GPS functions

TinyGPSPlus gps;               //Object gps for class TinyGPSPlus

void setup()
{
  Serial.begin(9600);         //Begins Serial comunication at Serial Port 1 at 9600 baudrate
}

void loop()
{
  GPSDelay(1000);
  double lat_val, lng_val;
  bool loc_valid;
  lat_val = gps.location.lat();        //Gets the latitude
  loc_valid = gps.location.isValid(); 
  lng_val = gps.location.lng();        //Gets the longitude
 
  if (!loc_valid)                     //Excecutes when invalid data is received from GPS
  {
    Serial.print("Latitude : ");
    Serial.println("*****");
    Serial.print("Longitude : ");
    Serial.println("*****");
    delay(4000);
  }
  else                              //Excutes when valid data is received from GPS
  {    
    Serial.println("GPS READING: ");
   
    Serial.print("Latitude : ");
    Serial.println(lat_val, 6);   //Prints latitude at Serial Monitor

    Serial.print("Longitude : ");
    Serial.println(lng_val, 6);   //Prints longitude at Serial monitor

    delay(4000);
  }
}

static void GPSDelay(unsigned long ms)          //Delay for receiving data from GPS
{
  unsigned long start = millis();
  do
  {
    while (Serial.available()) 
    gps.encode(Serial.read());
  } while (millis() - start < ms);
}
