#include <AFMotor.h>
//#include <Adafruit_>/
#include <HMC5883L.h> 
//#include <MPU6050.h>/


                                             // the Adafruit Motor Shield Library ver. 1 https://learn.adafruit.com/adafruit-motor-shield/library-install
#include "Wire.h"                                                 // Used by I2C and HMC5883L compass
//#include "I2Cdev.h"                                             // I2C Communications Library (used for compass)
                                  
#include <TinyGPS++.h>                                            // Tiny GPS Plus Library - Download from http://arduiniana.org/libraries/tinygpsplus/
                                                                  // TinyGPS++ object uses Hardware Serial 2 and assumes that you have a
                                                                  // 9600-baud serial GPS device hooked up on pins 16(tx) and 17(rx).                                                                  

//******************************************************************************************************                                                                  
// GPS Variables & Setup

int GPS_Course;                                                    // variable to hold the gps's determined course to destination
int Number_of_SATS;                                                // variable to hold the number of satellites acquired
TinyGPSPlus gps;                                                   // gps = instance of TinyGPS 
                                                                   // pin 17 (blue)   is connected to the TX on the GPS
                                                                   // pin 16 (yellow) is connected to the RX on the GPS

//******************************************************************************************************
// Setup Drive Motors using the Adafruit Motor Controller version 1.0 Library

AF_DCMotor motor1(1, MOTOR34_64KHZ);                               // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR34_64KHZ);                               // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_64KHZ);                               // create motor #3, 64KHz pwm
AF_DCMotor motor4(4, MOTOR34_64KHZ);                               // create motor #4, 64KHz pwm

int turn_Speed = 175;                                              // motor speed when using the compass to turn left and right
int mtr_Spd = 250;                                                 // motor speed when moving forward and reverse

//******************************************************************************************************
// Compass Variables & Setup

HMC5883L compass;                                                  // HMC5883L compass(HMC5883L)
int16_t mx, my, mz;                                                // variables to store x,y,z axis from compass (HMC5883L)
int desired_heading;                                               // initialize variable - stores value for the new desired heading
int compass_heading;                                               // initialize variable - stores value calculated from compass readings
int compass_dev = 5;                                               // the amount of deviation that is allowed in the compass heading - Adjust as Needed
                                                                   // setting this variable too low will cause the robot to continuously pivot left and right
                                                                   // setting this variable too high will cause the robot to veer off course

int Heading_A;                                                     // variable to store compass heading
int Heading_B;                                                     // variable to store compass heading in Opposite direction
int pass = 0;                                                      // variable to store which pass the robot is on

//******************************************************************************************************
// Servo Control

                                                    // create servo object to control a servo 
int pos = 0;                                                       // variable to store the servo position

//******************************************************************************************************
// Ping Sensor for Collision Avoidance

boolean pingOn = false;                                            // Turn Collision detection On or Off

int trigPin = 43;                                                  // Trig - Orange
int echoPin = 42;                                                  // Echo - Yellow
long duration, inches;
int Ping_distance;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;                                  // Store last time Ping was updated
const long interval = 200;                                         // Ping the Distance every X miliseconds
 
//******************************************************************************************************
// Bluetooth Variables & Setup

String str;                                                        // raw string received from android to arduino
int blueToothVal;                                                  // stores the last value sent over via bluetooth
int bt_Pin = 34;                                                   // Pin 34 of the Aruino Mega used to test the Bluetooth connection status - Not Used

//*****************************************************************************************************
// GPS Locations

unsigned long Distance_To_Home;                                    // variable for storing the distance to destination

int ac =0;                                                         // GPS array counter
int wpCount = 0;                                                   // GPS waypoint counter
double Home_LATarray[50];                                          // variable for storing the destination Latitude - Only Programmed for 5 waypoint
double Home_LONarray[50];                                          // variable for storing the destination Longitude - up to 50 waypoints


int increment = 0;

void setup() 
{  
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer
  Serial1.begin(9600);                                             // Serial 1 is for Bluetooth communication - DO NOT MODIFY - JY-MCU HC-06 v1.40
  Serial2.begin(9600);                                             // Serial 2 is for GPS communication at 9600 baud - DO NOT MODIFY - Ublox Neo 6m 
//  myservo.attach(9);                                               // attaches the servo to pin 9 (Servo 0 on the Adafruit Motor Control Board)
  
  pinMode(36, OUTPUT);                                             // define pin 36 as an output for an LED indicator - Not Used
  pinMode(bt_Pin, INPUT);                                          // This pin(34) is used to check the status of the Bluetooth connection - Not Used

  // Ping Sensor
//  pinMode(trigPin, OUTPUT);                                        // Ping Sensor
//  pinMode(echoPin, INPUT);                                         // Ping Sensor

  // Compass
  Wire.begin();                                                    // Join I2C bus used for the HMC5883L compass
  compass.begin();                                                 // initialize the compass (HMC5883L)
  compass.setRange(HMC5883L_RANGE_1_3GA);                          // Set measurement range  
  compass.setMeasurementMode(HMC5883L_CONTINOUS);                  // Set measurement mode  
  compass.setDataRate(HMC5883L_DATARATE_30HZ);                     // Set data rate  
  compass.setSamples(HMC5883L_SAMPLES_8);                          // Set number of samples averaged  
  compass.setOffset(0,0);                                          // Set calibration offset 

  Startup();                                                       // Run the Startup procedure on power-up one time
}

//********************************************************************************************************
// Main Loop

void loop()
{ 
  
  bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT                                                    
  getGPS();                                                        // Update the GPS location
  getCompass();                                                    // Update the Compass Heading
  Ping();                                                          // Use at your own discretion, this is not fully tested

}








//**************************************************************************************************************************************************
// This procedure reads the serial port - Serial1 - for bluetooth commands being sent from the Android device

void bluetooth()
{
 while (Serial1.available())                                    // Read bluetooth commands over Serial1 // Warning: If an error with Serial1 occurs, make sure Arduino Mega 2560 is Selected
 {  
  {  
      str = Serial1.readStringUntil('\n');                      // str is the temporary variable for storing the last sring sent over bluetooth from Android device
      //Serial.print(str);                                      // for testing purposes
  } 
    
    blueToothVal = (str.toInt());                               //  convert the string 'str' into an integer and assign it to blueToothVal
    Serial.print("BlueTooth Value ");
    Serial.println(blueToothVal);    

// **************************************************************************************************************************************************

 switch (blueToothVal) 
 {
      case 1:                                
        Serial1.println("Forward");
        Forward();
        break;

      case 2:                 
        Serial1.println("Reverse");
        Reverse();
        break;

      case 3:         
        Serial1.println("Left");
        LeftTurn();
        StopCar();
        break;
        
      case 4:                     
        Serial1.println("Right");
        RightTurn();
        StopCar();
        break;
        
      case 5:                                            
        Serial1.println("Stop Car ");
        StopCar();
        break; 

      case 6:                 
        setWaypoint();
        break;
      
      case 7:        
        goWaypoint();
        break;  
      
      case 8:        
        Serial1.println("Turn Around");
        turnAround();
        break;
      
      case 9:        
         Serial1.println("Compass Forward");
        setHeading();
        Compass_Forward();
        break;
      
      case 10:
        setHeading();
        break; 

      case 11:
         gpsInfo();
        break;
      
      case 12:  
        Serial1.println("Compass Turn Right");
        CompassTurnRight();
        break;
      
      case 13:  
        Serial1.println("Compass Turn Left");
        CompassTurnLeft();
        break;
        
      case 14:  
        Serial1.println("Calibrate Compass");
        calibrateCompass();
        break;

      case 15:  
        pingToggle();
        break;  

      case 16:
        clearWaypoints();
        break;  

      case 17:                    // finish with waypoints
        ac = 0;
        Serial1.print("Waypoints Complete");
        break;
      

 } // end of switch case

// **************************************************************************************************************************************************  
// Slider Value for Speed

if (blueToothVal)                                    
  {    
   //Serial.println(blueToothVal);
  if (blueToothVal >= 1000)
{
    Serial1.print("Speed set To:  ");
    Serial1.println(blueToothVal - 1000);
    turn_Speed = (blueToothVal - 1000); 
    Serial.println();
    Serial.print("Turn Speed ");
    Serial.println(turn_Speed);
 } 
  }  

/*

// **************************************************************************************************************************************************
// Detect for Mines - Sweep Not Used

 else if (blueToothVal== 15)                                   
  {    
    Serial1.println("Detecting");
    sweep();
  }

// **************************************************************************************************************************************************  
*/
 }                                                              // end of while loop Serial1 read
 
                                                               // if no data from Bluetooth 
   if (Serial1.available() < 0)                                 // if an error occurs, confirm that the arduino mega board is selected in the Tools Menu
    {
     Serial1.println("No Bluetooth Data ");          
    }
  
} // end of bluetooth procedure



void Ping()
{ 
  currentMillis = millis();
  
 if ((currentMillis - previousMillis >= interval) && (pingOn == true)) 
 {
  previousMillis = currentMillis;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  
  int inches = (duration / 2) / 74;                       // convert the time into a distance
  Ping_distance == inches;  
 
    if ((inches < 12) && (blueToothVal != 5))
      {
        Serial1.print("Crash! ");
        Serial1.println(inches);
        Reverse();                                        // Quick reverse to Stop quickly
        delay(100);
        StopCar();   
        blueToothVal = 5;                                 // Set bluetooth value to "Stop"
      } 
  
  }       // end if statement
 
}         // end Ping()

void pingToggle()                                         // Turns Collision avoidance on/ off
 {
  if (pingOn == true) {
    pingOn = false;
    Serial1.print("Collision Avoidance OFF");
  }
    else if (pingOn == false) {
    pingOn = true;
    Serial1.print("Collision Avoidance ON");
  }
  
 }



void calibrateCompass()                                           // Experimental Use Only to Calibrate Magnetometer/ Compass
{
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  int offX = 0;
  int offY = 0; 

  for (int i=1000; i >= 1; i--) 
  {
  Vector mag = compass.readRaw();                                 // Read compass data
  
  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;
  
  offX = (maxX + minX)/2;                                         // Calculate offsets
  offY = (maxY + minY)/2;
  
  delay(10);
  //Serial.print("Compass X & Y offset: ");
  //Serial.print(offX);
  //Serial.print(" ");
  //Serial.print(offY);
  //Serial.print("\n");
  
  }                                                               // end of for loop
  
  StopCar();

  Serial1.print("Compass X & Y offset: ");
  Serial1.print(offX);
  Serial1.print(" ");
  Serial1.print(offY);
  Serial.print("\n");
  compass.setOffset(offX,offY);                                  // Set calibration offset
}
 
 // ************************************************************************************************************************************************* 

void getGPS()                                                 // Get Latest GPS coordinates
{
    while (Serial2.available() > 0)
    gps.encode(Serial2.read());
} 

// *************************************************************************************************************************************************
 
void setWaypoint()                                            // Set up to 5 GPS waypoints
{

//if ((wpCount >= 0) && (wpCount < 50))
if (wpCount >= 0)
  {
    Serial1.print("GPS Waypoint ");
    Serial1.print(wpCount + 1);
    Serial1.print(" Set ");
    getGPS();                                                 // get the latest GPS coordinates
    getCompass();                                             // update latest compass heading     
                                               
    Home_LATarray[ac] = gps.location.lat();                   // store waypoint in an array   
    Home_LONarray[ac] = gps.location.lng();                   // store waypoint in an array   
                                                              
    Serial.print("Waypoint #1: ");
    Serial.print(Home_LATarray[0],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0],6);
    Serial.print("Waypoint #2: ");
    Serial.print(Home_LATarray[1],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[1],6);
    Serial.print("Waypoint #3: ");
    Serial.print(Home_LATarray[2],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[2],6);
    Serial.print("Waypoint #4: ");
    Serial.print(Home_LATarray[3],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[3],6);
    Serial.print("Waypoint #5: ");
    Serial.print(Home_LATarray[4],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[4],6);

    wpCount++;                                                  // increment waypoint counter
    ac++;                                                       // increment array counter
        
  }         
  else {Serial1.print("Waypoints Full");}
}

// ************************************************************************************************************************************************* 

void clearWaypoints()
{
   memset(Home_LATarray, 0, sizeof(Home_LATarray));             // clear the array
   memset(Home_LONarray, 0, sizeof(Home_LONarray));             // clear the array
   wpCount = 0;                                                 // reset increment counter to 0
   ac = 0;
   
   Serial1.print("GPS Waypoints Cleared");                      // display waypoints cleared
  
}

 // *************************************************************************************************************************************************
 
void getCompass()                                               // get latest compass value
 {  

  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
 
  if(heading < 0)
     heading += 2 * M_PI;      
  compass_heading = (int)(heading * 180/M_PI);                   // assign compass calculation to variable (compass_heading) and convert to integer to remove decimal places                                                              

 }

 // *************************************************************************************************************************************************

void setHeading()
                                                                 // This procedure will set the current heading and the Heading(s) of the robot going away and returning using opposing degrees from 0 to 360;
                                                                 // for instance, if the car is leaving on a 0 degree path (North), it will return on a 180 degree path (South)
{
   for (int i=0; i <= 5; i++)                                    // Take several readings from the compass to insure accuracy
      { 
        getCompass();                                            // get the current compass heading
      }                                               
    
    desired_heading = compass_heading;                           // set the desired heading to equal the current compass heading
    Heading_A = compass_heading;                                 // Set Heading A to current compass 
    Heading_B = compass_heading + 180;                           // Set Heading B to current compass heading + 180  

      if (Heading_B >= 360)                                      // if the heading is greater than 360 then subtract 360 from it, heading must be between 0 and 360
         {
          Heading_B = Heading_B - 360;
         }
     
    Serial1.print("Compass Heading Set: "); 
    Serial1.print(compass_heading);   
    Serial1.print(" Degrees");

    Serial.print("desired heading");
    Serial.println(desired_heading);
    Serial.print("compass heading");
    Serial.println(compass_heading);

}

// *************************************************************************************************************************************************
 
void gpsInfo()                                                  // displays Satellite data to user
  {
        Number_of_SATS = (int)(gps.satellites.value());         //Query Tiny GPS for the number of Satellites Acquired 
        Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination    
        Serial1.print("Lat:");
        Serial1.print(gps.location.lat(),6);
        Serial1.print(" Lon:");
        Serial1.print(gps.location.lng(),6);
        Serial1.print(" ");
        Serial1.print(Number_of_SATS); 
        Serial1.print(" SATs ");
        Serial1.print(Distance_To_Home);
        Serial1.print("m"); 
Serial.print("Distance to Home ");
Serial.println(Distance_To_Home);
  
  }
  
  
void goWaypoint()
{   
 Serial1.println("Go to Waypoint");
//Serial.print("Home_Latarray ");
//Serial.print(Home_LATarray[ac],6);
//Serial.print(" ");
//Serial.println(Home_LONarray[ac],6);   

//Serial1.print("Distance to Home");   
//Serial1.print(Distance_To_Home);

//Serial1.print("ac= ");
//Serial1.print(ac);

 while (true)  
  {                                                                // Start of Go_Home procedure 
  bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
  if (blueToothVal == 5){break;}                                   // If a 'Stop' Bluetooth command is received then break from the Loop
  getCompass();                                                    // Update Compass heading                                          
  getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location// delay time changed from 100 to 10
  
  if (millis() > 5000 && gps.charsProcessed() < 10)                // If no Data from GPS within 5 seconds then send error
    Serial1.println(F("No GPS data: check wiring"));     
 
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
  GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),Home_LATarray[ac],Home_LONarray[ac]);                               //Query Tiny GPS for Course to Destination   
   
   /*
    if (Home_LATarray[ac] == 0) {
      Serial1.print("End of Waypoints");
      StopCar();      
      break;
      }      
   */ 
    if (Distance_To_Home == 0)                                   // If the Vehicle has reached it's Destination, then Stop
        {
        StopCar();                                               // Stop the robot after each waypoint is reached
        Serial1.println("You have arrived!");                    // Print to Bluetooth device - "You have arrived"          
        ac++;                                                    // increment counter for next waypoint
        break;                                                   // Break from Go_Home procedure and send control back to the Void Loop 
                                                                 // go to next waypoint
        
        }   
   
   
   if ( abs(GPS_Course - compass_heading) <= 15)                  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward                                                                  
                                                                  // otherwise find the shortest turn radius and turn left or right  
       {
         Forward();                                               // Go Forward
       } else 
         {                                                       
            int x = (GPS_Course - 360);                           // x = the GPS desired heading - 360
            int y = (compass_heading - (x));                      // y = the Compass heading - x
            int z = (y - 360);                                    // z = y - 360
            
            if ((z <= 180) && (z >= 0))                           // if z is less than 180 and not a negative value then turn left otherwise turn right
                  { SlowLeftTurn();  }
             else { SlowRightTurn(); }               
        } 
    

  }                                                              // End of While Loop

  
}                                                                // End of Go_Home procedure
void Startup()
{
//     /myservo.detach(); 
     Serial.println("Pause for Startup... ");
             
     for (int i=5; i >= 1; i--)                       // Count down for X seconds
      {         
        Serial1.print("Pause for Startup... "); 
        Serial1.print(i);
        delay(1000);                                   // Delay for X seconds
      }    
    
  Serial1.println("Searching for Satellites "); 
  Serial.println("Searching for Satellites "); 
      
  while (Number_of_SATS <= 4)                         // Wait until x number of satellites are acquired before starting main loop
  {                                  
    getGPS();                                         // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired       
    bluetooth();                                      // Check to see if there are any bluetooth commands being received     
  }    
  setWaypoint();                                      // set intial waypoint to current location
  wpCount = 0;                                        // zero waypoint counter
  ac = 0;                                             // zero array counter
  
  Serial1.print(Number_of_SATS);
  Serial1.print(" Satellites Acquired");    
}    


void Forward()
{
  Ping();
  Serial.println("Forward");
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);                           
  
  motor1.run(FORWARD);                                                         // go forward all wheels 
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

// **********************************************************************************************************************************************************************

void Forward_Meter()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);     
  
  motor1.run(FORWARD);                                                        // go forward all wheels for specified time
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
}

// **********************************************************************************************************************************************************************

void Reverse()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                        // Reverse all wheels
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// **********************************************************************************************************************************************************************

void LeftTurn()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                        // Turn left
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  delay(100);    
}

// **********************************************************************************************************************************************************************

void RightTurn()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);   

  motor1.run(FORWARD);                                              
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  delay(100);                                                                   //delay for 100ms more responsive turn per push on bluetooth  
}

// **********************************************************************************************************************************************************************


void SlowLeftTurn()
{
   
  motor1.setSpeed(turn_Speed);                                                
  motor2.setSpeed(turn_Speed);                      
  motor3.setSpeed(turn_Speed);                       
  motor4.setSpeed(turn_Speed);        
  
  motor1.run(BACKWARD);                         
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

// **********************************************************************************************************************************************************************

// Turning too fast will over-shoot the compass and GPS course


void SlowRightTurn()
{
   
  motor1.setSpeed(turn_Speed);                                                  
  motor2.setSpeed(turn_Speed);                      
  motor3.setSpeed(turn_Speed);                       
  motor4.setSpeed(turn_Speed);         

  motor1.run(FORWARD);                                                           
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

// **********************************************************************************************************************************************************************

void StopCar()
{
   
  
  motor1.run(RELEASE);                                                         
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
   
}


// Compass Drive Section
// This Portion of code steers the Vehicle based on the compass heading
// **********************************************************************************************************************************************************************

void CompassTurnRight()                                                          // This Function Turns the car 90 degrees to the right based on the current desired heading
{
  StopCar();    
  getCompass();                                                                  // get current compass heading      

  desired_heading = (desired_heading + 90);                                      // set desired_heading to plus 90 degrees 
  if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}       // if the desired heading is greater than 360 then subtract 360 from it

  while ( abs(desired_heading - compass_heading) >= compass_dev)                 // If the desired heading is more than Compass Deviation in degrees from the actual compass heading then
      {                                                                          // correct direction by turning left or right

    getCompass();                                                                // Update compass heading during While Loop
    bluetooth();                                                                 // if new bluetooth value received break from loop        
    if (blueToothVal == 5){break;}                                               // If a Stop Bluetooth command ('5') is received then break from the Loop
        
    if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}     // if the desired heading is greater than 360 then subtract 360 from it                                            
                                                                
    int x = (desired_heading - 359);                                             // x = the GPS desired heading - 360    
    int y = (compass_heading - (x));                                             // y = the Compass heading - x
    int z = (y - 360);                                                           // z = y - 360
            
        if ((z <= 180) && (z >= 0))                                              // if z is less than 180 and not a negative value then turn left 
            {                                                                    // otherwise turn right
              SlowLeftTurn();                            
            } 
            else
            {
              SlowRightTurn();        
            }  
        }    
    {
      StopCar();                                                                  // Stop the Car when desired heading and compass heading match
    }
 }    


// **********************************************************************************************************************************************************************

void CompassTurnLeft()                                                           // This procedure turns the car 90 degrees to the left based on the current desired heading
{
  StopCar();    
  getCompass();                                                                  // get current compass heading                                                                                  
  //desired_heading = (compass_heading - 90);                                    // set desired_heading to compass value minus 90 degrees

  desired_heading = (desired_heading - 90);                                      // set desired_heading to minus 90 degrees
  if (desired_heading <= 0) {desired_heading = (desired_heading + 360);}         // if the desired heading is greater than 360 then add 360 to it
  while ( abs(desired_heading - compass_heading) >= compass_dev)                 // If the desired heading is more than Compass Deviation in degrees from the actual compass heading then
      {                                                                          // correct direction by turning left or right
    getCompass();                                                                // Get compass heading again during While Loop
    bluetooth();                                                                 // if new bluetooth value received break from loop              
    if (blueToothVal == 5){break;}                                               // If a 'Stop' Bluetooth command is received then break from the Loop
    
    if (desired_heading >= 360) {desired_heading = (desired_heading - 360);}     // if the desired heading is greater than 360 then subtract 360 from it                                            
                                                                
    int x = (desired_heading - 359);                                             // x = the desired heading - 360    
    int y = (compass_heading - (x));                                             // y = the Compass heading - x
    int z = (y - 360);                                                           // z = y - 360
        if (z <= 180)                                                            // if z is less than 180 and not a negative value then turn left         
       // if ((z <= 180) && (z >= 0))                                             
            {                                                                    // otherwise turn right
              SlowLeftTurn();                             
            } 
            else
            {
              SlowRightTurn();              
            }  
        }    
    {
      StopCar();                                                                 // Stop the Car when desired heading and compass heading match
    }
 }   

// **********************************************************************************************************************************************************************

void Compass_Forward()                                               
{            
  while (blueToothVal == 9)                                           // Go forward until Bluetooth 'Stop' command is sent

  //while (true)                                                        
   {  
    getCompass();                                                     // Update Compass Heading
    bluetooth();                                                      // Check to see if any Bluetooth commands have been sent
    if (blueToothVal == 5) {break;}                                   // If a Stop Bluetooth command ('5') is received then break from the Loop
    
    if ( abs(desired_heading - compass_heading) <= compass_dev )      // If the Desired Heading and the Compass Heading are within the compass deviation, X degrees of each other then Go Forward
                                                                      // otherwise find the shortest turn radius and turn left or right  
       {
         Forward(); 
         Ping();       
       } else 
         {    
            int x = (desired_heading - 359);                          // x = the GPS desired heading - 360
            int y = (compass_heading - (x));                          // y = the Compass heading - x
            int z = (y - 360);                                        // z = y - 360
                     
            if ((z <= 180) && (z >= 0))                               // if z is less than 180 and not a negative value then turn left 
            {                                                         // otherwise turn right
              SlowLeftTurn();
              Ping();           
            }
            else
            {
              SlowRightTurn();
              Ping(); 
            }              
        } 
 }                                                                   // End While Loop
}                                                                    // End Compass_Forward

// **********************************************************************************************************************************************************************

void turnAround()                                                   // This procedure turns the Car around 180 degrees, every time the "Turn Around" button is pressed
 {                                                                  // the car alternates whether the next turn will be to the left or right - this is determined by the 'pass' variable
                                                                    // Imagine you are cutting the grass, when you get to the end of the row - the first pass you are turning one way and on the next pass you turn the opposite   
    if (pass == 0) { CompassTurnRight(); }                          // If this is the first pass then turn right
    
    else { CompassTurnLeft(); }                                     // If this is the second pass then turn left
      
    //Forward_Meter();                                              // Go forward one meter (approximately)
    StopCar();                                                      // Stop the car
    
       
    if (pass == 0)                                                  // If this is the first pass then Turn Right
      {       
        CompassTurnRight();                                         // Turn right
        pass = 1 ;                                                  // Change the pass value to '1' so that the next turn will be to the left
      }
      
    else 
      {     
         
    if (desired_heading == Heading_A)                               // This section of code Alternates the desired heading 180 degrees
     {                                                              // for the Compass drive forward
      desired_heading = Heading_B;
     }
    else if (desired_heading == Heading_B)
     {
      desired_heading = Heading_A;
     }        
          
        CompassTurnLeft();                                          // If this is the second pass then Turn Left
        pass = 0;                                                   // Change the pass value to '0' so that the next turn will be to the right

      }
      
  Compass_Forward();                                                // Maintain the 'desired heading' and drive forward
}
   
    

void sweep()                          // Can be used to simulate Metal Detecting or to rotate Ping Sensor
{ 
//  /myservo.attach(9); 
  StopCar();
  Forward_Meter();
  StopCar();
    
  for(pos = 60; pos <= 120; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                   // in steps of 1 degree 
//    /myservo.write(pos);               // tell servo to go to position in variable 'pos' 
    delay(15);                        // waits 15ms for the servo to reach the position 
  } 
  for(pos = 120; pos>=60; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
//    /myservo.write(pos);               // tell servo to go to position in variable 'pos' 
    delay(15);                        // waits 15ms for the servo to reach the position 
  } 

//    /myservo.write(90);                // tell servo to go to position in variable 'pos' 
    delay(15);                        // waits 15ms for the servo to reach the position 

//   /myservo.detach();                  // Disengage Servo Motor
} 
