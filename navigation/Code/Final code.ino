#include <AFMotor.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

char command;

void setup()
{
  Serial.begin (9600);
  Serial2.begin(9600);

  Serial.println(F("GPS coordinates for Jibebe 2023"));
  Serial.println();
  Serial.println(F(" Latitude Longitude  Altitude  Metres"));
  }

void loop ()
{
  static const double TARGET_LAT = -1.0955774250381571, TARGET_LON =37.01194398382244;

  //printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  //printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  //printInt(gps.location.age(), gps.location.isValid(), 5);
  //printDateTime(gps.date, gps.time);
  //printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  //printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  //printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  //printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);  
  
   unsigned long distanceKmToTarget =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      TARGET_LAT, 
      TARGET_LON) / 1000;
  //printInt(distanceKmToTarget, gps.location.isValid(), 9);

  double courseToTarget =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      TARGET_LAT, 
      TARGET_LON);
printFloat(courseToTarget, gps.location.isValid(), 6, 1);

  const char *cardinalToTarget = TinyGPSPlus::cardinal(courseToTarget);

  printStr(gps.location.isValid() ? cardinalToTarget : "*** ", 7);

  //printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  //printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
  if (Serial2.available() > 0){
    command = Serial2.read();
    Stop();
    switch(command){
      case 'TARGET_LON, TARGET_LAT':
        forward();
        left();
        right();
        break;              

    }
  
  }
}


// This custom version of delay() ensures that the gps object
// is being "fed".
String data; char data_;
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available ()){
      data_ = Serial2.read() ;
    // while (ss.available()){
    //   data_ = ss.read() ;  
      // data = data + data_;
      #ifdef Test_GPS
        Serial.print(data_);  
      #endif
      gps.encode(data_);      
    } 
  } while (millis() - start < ms);
}      

  //if(serial.available()> 10){
    //command = serial2.read();
    //Stop();   
//}


void forward()
{
 motor1.setSpeed(255); //Define maximum velocity
 motor1.run(FORWARD); //rotate the motor clockwise
 motor2.setSpeed(255); //Define maximum velocity
 motor2.run(FORWARD); //rotate the motor clockwise 
 motor3.setSpeed(255); //Define maximum velocity
 motor3.run(FORWARD); //rotate the motor clockwise
 motor4.setSpeed(255); //Define maximum velocity
 motor4.run(FORWARD); //rotate the motor clockwise
}

void back()
{
 motor1.setSpeed(255); 
 motor1.run(BACKWARD); //rotate the motor counterclockwise
 motor2.setSpeed(255); 
 motor2.run(BACKWARD); //rotate the motor counterclockwise

 motor3.setSpeed(255); 
 motor3.run(BACKWARD); //rotate the motor counterclockwise
 motor4.setSpeed(255); 
 motor4.run(BACKWARD); //rotate the motor counterclockwise
}

void left()
{
 motor1.setSpeed(255); //Define maximum velocity
 motor1.run(FORWARD); //rotate the motor clockwise
 motor2.setSpeed(255); //Define maximum velocity
 motor2.run(BACKWARD); //rotate the motor counterclockwise

 motor3.setSpeed(255); //Define maximum velocity
 motor3.run(FORWARD); //rotate the motor clockwise
 motor4.setSpeed(255); //Define maximum velocity
 motor4.run(BACKWARD); //rotate the motor counterclockwise
}

void right()
{
 motor1.setSpeed(255); //Define maximum velocity
 motor1.run(BACKWARD); //rotate the motor counterclockwise
 motor2.setSpeed(255); //Define maximum velocity
 motor2.run(FORWARD); //rotate the motor clockwise

 motor3.setSpeed(255); //Define maximum velocity
 motor3.run(BACKWARD); //turn motor1 off
 motor4.setSpeed(255); //Define maximum velocity
 motor4.run(FORWARD); //rotate the motor clockwise
}

void Stop()
{
 motor1.setSpeed(0);
 motor2.run(RELEASE); //turn motor1 off
 motor2.setSpeed(0);
 motor2.run(RELEASE); //turn motor2 off

 motor3.setSpeed(0);
 motor3.run(RELEASE); //turn motor3 off
 motor4.setSpeed(0);
 motor4.run(RELEASE); //turn motor4 off
}


static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}
static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}
