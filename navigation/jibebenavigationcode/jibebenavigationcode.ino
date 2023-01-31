

#include <HCSR04.h>//Ulrasonic sensor funtion library
#include <AFMotor.h>//motor drive module library

//Code
/* Obstacle Avoiding Robot Using Ultrasonic Sensor and Ard*/
#include <Servo.h>  //Servo motor library
//#include <HCSR04.h> 

int trigPin = 36;      // trig pin of HC-SR04 .. assigning output pins to ultrasonic sensor
int echoPin = 37;     // Echo pin of HC-SR04







long duration; //declaration of variables
#define maximum_distance 200
int distance;
int distance1;
int pos = 0;
int distanceRight;
int distanceLeft;
//int lookRight;
//int lookLeft;
Servo myservo;
HCSR04 sonar(trigPin, echoPin, maximum_distance); //sensor function


AF_DCMotor motor1(1, MOTOR12_64KHZ); 
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR34_64KHZ); //assignment and initialization of motor pins .. the first argument assigns what pins of the driver are used
AF_DCMotor motor4(4, MOTOR34_64KHZ); //second argument is used to set the frequency


void setup() { //this function sets immutable objects. those that remain constant throughout the program.
  

  Serial.begin(9600);// serial communication baud rate setting
  motor1.setSpeed(80);//setting the speed of the motors
  motor2.setSpeed(80);
  motor3.setSpeed(80);
  motor4.setSpeed(80);
//  motor1.run(RELEASE);
//  motor2.run(RELEASE);
  
  pinMode(trigPin, OUTPUT);         // set trig pin as output
  pinMode(echoPin, INPUT);          //set echo pin as input to capture reflected waves

  myservo.attach(10); //servo pin
  myservo.write(90); //this is the forward lookout of our servo motor.. with left being 0 and right being 180 degrees

  delay(100);

}

void loop() { //this function is done repeatedly
  // we start by checking whether there's an obstacle ahead
//  myservo.write(40);
  
  distance = readPing();
  
//  delay(500);
//  myservo.write(90);
//  distance = readPing();
//  delay(500);
//  myservo.write(140);
//  distance = readPing();
//  delay(500);
//  myservo.write(90);
//  distance = readPing();
//  delay(500);
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);   
//  digitalWrite(trigPin, HIGH);     // send waves for 10 us
//  delayMicroseconds(10);
//  duration = pulseIn(echoPin, HIGH); // receive reflected waves
//  Serial.println(duration);
//  Serial.println(duration);
//  distance = duration / 58.2; // convert to distance
//  distance = readPing();
  Serial.println(distance);
  
   
  delay(10);
    
  if (distance > 30)            
  {
  Serial.println("moving forward");
  motor1.run(FORWARD);//move forward
  motor2.run(FORWARD);        
  motor3.run(FORWARD);//move forward
  motor4.run(FORWARD);                                                       
  }

  else if (distance<19) // if the distance between sensor and obstacle is greater than 19
  {  
    Serial.println("obstacle detected");
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    delay(200);

    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);

    delay(300);

    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    delay(500);
 
    distanceRight = lookRight(); //function to look right is called that returns an int type distance
    Serial.println(distanceRight);
   
    delay(300);
    distanceLeft = lookLeft();
    Serial.println(distanceRight);
    delay(300);

    if (distanceRight >= distanceLeft){ // if the distance to the right is greater than that to the left then turn right
      Serial.println("turning right");
      turnRight();// function to turn right is called. 
      delay(50);
      
      moveForward();
//      motor1.run(RELEASE);
//      motor2.run(RELEASE);
    }
    else{
      
      Serial.println("turning left");
      turnLeft();
      delay(50);
//      moveForward();
//      moveStop();
    }
  }
  
   
  
 

}

// turning left and right coding
void turnLeft(){
//    motor1.run(BACKWARD);
//    motor2.run(BACKWARD);
//    motor3.run(BACKWARD);
//    motor4.run(BACKWARD);
//    delay(300); 
    motor1.run(BACKWARD);
    motor4.run(BACKWARD);// stop the right motor
    motor2.run(FORWARD);
    motor3.run(FORWARD);// run the left motor
    delay(700);

    motor1.run(RELEASE); // stop motors
    motor2.run(RELEASE);
    motor3.run(RELEASE); // stop motors
    motor4.run(RELEASE);
    
  


}

void turnRight (){
//    motor1.run(BACKWARD);
//    motor2.run(BACKWARD);
//    delay(300);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);// stop the right motor
    motor1.run(FORWARD);
    motor4.run(FORWARD);// run the left motor
    delay(700);
    
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE); // stop motors
    motor4.run(RELEASE);
    
    
  
}
void moveStop(){
  motor1.run(RELEASE); // stop motors
  motor2.run(RELEASE);
  
}
void moveForward(){ // function to move forward
  motor1.run(FORWARD);  
  motor2.run(FORWARD);
}
void moveBackward(){ // function to move backward
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
}
int readPing(){ // function to detect distance
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);   
  digitalWrite(trigPin, HIGH);     // send waves for 10 us
  delayMicroseconds(10);
  duration = pulseIn(echoPin, HIGH); // receive reflected waves
  Serial.println("readping works");
  distance1 = duration / 58.2;

  return distance1;
}
int lookRight(){ // function to turn servo motor to the right and get distance by calling the above function
  myservo.write(150); //turn right
  delay(500);
  int distance = readPing(); //get distance
  delay(100);
  myservo.write(90);
  
  return distance; // return to caller the distance to the right
  delay(100);
}
int lookLeft(){ // same as above. only to the left
  myservo.write(30);
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(90);
  return distance;
  delay(100);
}
void gpsdata()
{
 if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print(F("- latitude: "));
        Serial.println(gps.location.lat());

        Serial.print(F("- longitude: "));
        Serial.println(gps.location.lng());

        Serial.print(F("- altitude: "));
        if (gps.altitude.isValid())
          Serial.println(gps.altitude.meters());
        else
          Serial.println(F("INVALID"));
      } else {
        Serial.println(F("- location: INVALID"));
      }

      Serial.print(F("- speed: "));
      if (gps.speed.isValid()) {
        Serial.print(gps.speed.kmph());
        Serial.println(F(" km/h"));
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.println();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  Serial.println(F("No GPS data received: check wiring"));
  latc= latitude;
  logc=  longitude;

}  

void gpsheading()
{
  float x,y,deltalog,deltalat;
  deltalog= logd-logc;
  deltalat=latd-latc;

  x=cos(latd)*sin(deltalog);
  y=(cos(latc)*sin(latd))-(sin(latc)*cos(latd)*cos(deltalog));
  
  bearing=(atan2(x,y))*(180/3.14);
  Serial.print("bearing");
  Serial.println(bearing);

  float a,d,c;
  a=(((sin(deltalat/2)))*(sin(deltalat/2))) + ((cos(latc))*(cos(latd))* (((sin(deltalog/2)))*(sin(deltalog/2)))  );
  c=2*(atan2(sqrt(a),sqrt(1-a)));
  d=6371*c; 
//Serial.println(d);  
 }
