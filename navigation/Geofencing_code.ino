#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <math.h>
#include <LiquidCrystal_I2C.h>

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

#define LATITUDE 0.0888 // Enter the latitude of the geofence boundary
#define LONGITUDE 37.6544 // Enter the longitude of the geofence boundary
#define RADIUS 100 // Enter the radius of the geofence boundary in meters


AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Define the GPS coordinates of the starting point
const float startLat = 0.08886;
const float startLng = 37.65450;

// Define the maximum distance from the starting point (in meters)
const float maxDistance = 15;
float flat, flon;
    unsigned long age;
     bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  // gpsSerial.begin(9600);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2, 0)  ;
  lcd.print("Jibebe Project");
  delay(1500);
}

void loop() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
       float lat, lng;
      unsigned long fix_age;
      gps.f_get_position(&lat, &lng, &fix_age);
      float distance = calculateDistance(startLat, startLng, lat, lng);
 
      
      if (distance > RADIUS) {
        // Out of geofence boundary
         lcd.clear();
      lcd.backlight();
      lcd.setCursor(2,0);
      lcd.println("Tumepotea");
        Serial.println("Out of boundary");
        forward();
        // Add code to stop the truck or sound an alarm here
      } else {// Inside geofence boundary
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(2,0);
      lcd.println("Tumefika");
      Serial.println("Inside boundary");
      stop();
        // Add code to continue the truck route here
      }
      lcd.init();
      lcd.clear();
      lcd.backlight();
      lcd.setCursor(1,0);
      gps.f_get_position(&flat, &flon, &age);
      lcd.setCursor(1, 0);
      lcd.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      lcd.print(" ");           
      lcd.print(distance);
      lcd.print(" M");
      lcd.setCursor(1,1);
      lcd.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6); 
      delay(300);
    
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
    }
  }
}
  float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371000; // Earth's radius in meters
  float dLat = (lat2 - lat1) * M_PI / 180;
  float dLng = (lng2 - lng1) * M_PI / 180;
  float a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * sin(dLng/2) * sin(dLng/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}


void forward() {
  motor1.setSpeed(100);  //Define maximum velocity
  motor1.run(FORWARD);   //rotate the motor clockwise
  motor2.setSpeed(100);  //Define maximum velocity
  motor2.run(FORWARD);   //rotate the motor clockwise
  motor3.setSpeed(100);  //Define maximum velocity
  motor3.run(FORWARD);   //rotate the motor clockwise
  motor4.setSpeed(100);  //Define maximum velocity
  motor4.run(FORWARD);   //rotate the motor clockwise
}
void forwardleft() {
  motor1.setSpeed(100);  //Define maximum velocity
  motor1.run(FORWARD);   //rotate the motor clockwise
  motor2.setSpeed(170);  //Define maximum velocity
  motor2.run(BACKWARD);  //rotate the motor clockwise
  motor3.setSpeed(100);  //Define maximum velocity
  motor3.run(FORWARD);   //rotate the motor clockwise
  motor4.setSpeed(100);  //Define maximum velocity
  motor4.run(FORWARD);   //rotate the motor clockwise
}
void forwardright() {
  motor1.setSpeed(170);  //Define maximum velocity
  motor1.run(BACKWARD);  //rotate the motor clockwise
  motor2.setSpeed(100);  //Define maximum velocity
  motor2.run(FORWARD);   //rotate the motor clockwise
  motor3.setSpeed(100);  //Define maximum velocity
  motor3.run(FORWARD);   //rotate the motor clockwise
  motor4.setSpeed(100);  //Define maximum velocity
  motor4.run(FORWARD);   //rotate the motor clockwise
}
void back() {
  motor1.setSpeed(100);
  motor1.run(BACKWARD);  //rotate the motor counterclockwise
  motor2.setSpeed(100);
  motor2.run(BACKWARD);  //rotate the motor counterclockwise

  motor3.setSpeed(100);
  motor3.run(BACKWARD);  //rotate the motor counterclockwise
  motor4.setSpeed(100);
  motor4.run(BACKWARD);  //rotate the motor counterclockwise
}
void backright() {
  motor1.setSpeed(170);
  motor1.run(FORWARD);  //rotate the motor counterclockwise
  motor2.setSpeed(100);
  motor2.run(BACKWARD);  //rotate the motor counterclockwise

  motor3.setSpeed(100);
  motor3.run(BACKWARD);  //rotate the motor counterclockwise
  motor4.setSpeed(100);
  motor4.run(BACKWARD);  //rotate the motor counterclockwise
}
void backleft() {
  motor1.setSpeed(100);
  motor1.run(BACKWARD);  //rotate the motor counterclockwise
  motor2.setSpeed(170);
  motor2.run(FORWARD);  //rotate the motor counterclockwise

  motor3.setSpeed(100);
  motor3.run(BACKWARD);  //rotate the motor counterclockwise
  motor4.setSpeed(100);
  motor4.run(BACKWARD);  //rotate the motor counterclockwise
}
void left() {
  motor1.setSpeed(170);  //Define maximum velocity
  motor1.run(FORWARD);   //rotate the motor clockwise
  motor2.setSpeed(170);  //Define maximum velocity
  motor2.run(BACKWARD);  //rotate the motor counterclockwise

  motor3.setSpeed(170);  //Define maximum velocity
  motor3.run(FORWARD);   //rotate the motor clockwise
  motor4.setSpeed(170);  //Define maximum velocity
  motor4.run(BACKWARD);  //rotate the motor counterclockwise
}

void right() {
  motor1.setSpeed(170);  //Define maximum velocity
  motor1.run(BACKWARD);  //rotate the motor counterclockwise
  motor2.setSpeed(170);  //Define maximum velocity
  motor2.run(FORWARD);   //rotate the motor clockwise

  motor3.setSpeed(170);  //Define maximum velocity
  motor3.run(BACKWARD);  //turn motor1 off
  motor4.setSpeed(170);  //Define maximum velocity
  motor4.run(FORWARD);   //rotate the motor clockwise
}

void stop() {
  motor1.setSpeed(0);
  motor2.run(RELEASE);  //turn motor1 off
  motor2.setSpeed(0);
  motor2.run(RELEASE);  //turn motor2 off

  motor3.setSpeed(0);
  motor3.run(RELEASE);  //turn motor3 off
  motor4.setSpeed(0);
  motor4.run(RELEASE);  //turn motor4 off
}
