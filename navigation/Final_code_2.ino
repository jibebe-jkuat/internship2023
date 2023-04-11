// It is possible to control a GPS-controlled robot based on distances calculated from GPS coordinates, but it may not be the most accurate or reliable method.

// To calculate distances between GPS coordinates, you can use the Haversine formula, which takes into account the curvature of the earth. Here's an example code that uses the Haversine formula to calculate the distance between two GPS coordinates:


#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <math.h>

// SoftwareSerial gpsSerial(2, 3);
TinyGPS gps;

// Define the GPS coordinates of the starting point
const float startLat = 0.0887;
const float startLng = 37.6544;

// Define the maximum distance from the starting point (in meters)
const float maxDistance = 100;
float flat, flon;
    unsigned long age;
     bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  // gpsSerial.begin(9600);
  
  Serial.println("Project by Dennis");
  Serial.println("Automation using GPS coordinates");
  Serial.println();  
}

void loop() { 
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      float lat, lng;
      unsigned long fix_age;
      gps.f_get_position(&lat, &lng, &fix_age);

      // Calculate the distance between the current GPS coordinates and the starting point
      float distance = calculateDistance(startLat, startLng, lat, lng);

      // Control the robot based on the distance from the starting point
      if (distance <= maxDistance) {
        // Move the robot forward
      } else {
        // Stop the robot
      }
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(" LAT  ");
    Serial.print("   LON  ");
    Serial.print("        Distance from starting point ");
        
    Serial.println();
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print("  ");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print("   ");
    Serial.print(distance);
    Serial.print(" meters");  
    Serial.println(); 
    Serial.println();   

 

  // // For one second we parse GPS data and report some key values
  // for (unsigned long start = millis(); millis() - start < 1000;)
  // {
  //   while (Serial2.available())
  //   {
  //     char c = Serial2.read();
  //     // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
  //     if (gps.encode(c)) // Did a new valid sentence come in?
  //       newData = true;
  //   }
  // }

 
//     Serial.print(" SAT=");
//     Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
//     Serial.print(" PREC=");
//     Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

//   gps.stats(&chars, &sentences, &failed);
//   Serial.print(" CHARS=");
//   Serial.print(chars);
//   Serial.print(" SENTENCES=");
//   Serial.print(sentences);
//   Serial.print(" CSUM ERR=");
//   Serial.println(failed);
//   if (chars == 0)
//     Serial.println("** No characters received from GPS: check wiring **");
}
 
    }
  }


// Function to calculate the distance between two GPS coordinates using the Haversine formula
float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371000; // Earth's radius in meters
  float dLat = (lat2 - lat1) * M_PI / 180;
  float dLng = (lng2 - lng1) * M_PI / 180;
  float a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * sin(dLng/2) * sin(dLng/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}


// In this code, we define
// PS I love you. And i asked the Ask AI app to write this for me. Get it for free --> https://get-askai.app