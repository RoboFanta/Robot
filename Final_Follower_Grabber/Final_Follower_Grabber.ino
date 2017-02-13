
// Final Follower - Final Code for following a line for up to 2 Arduinos,
// One of which may be controlling two motors based on a Gyro.


// Last Modified:
// 2017-02-06     For Uni Project.     Felix Karg


/*
This Arduino code will take inputs from a few lightsensors,
and communicates with a second arduino in completing the task of taking up a ball,
following a line, and dropping the ball at the designated point.


Input:
1 Light sensor for sensing the ball.
1 Ultrasonic sensor.

Output:
1 Motor for grabbing a ball.

Communication:
I2C Slave, Other arduino navigating and following the line.


Written mainly by   Felix Karg     <felix.karg@uranus.uni-freiburg.de>
improvements from   Paul Boeninger <>
              and   Victor Maier   <>

For trying a Arduino-style robot at the SDP Project, University of Freiburg, WS2016/17.

License is GPL-v3.
(included in subdirectory)
*/



#include <Wire.h>
#include "I2Cdev.h"
#include <NewPing.h>
#include <Servo.h>

#define DEBUG 1
// #define DEBUG 0

///////////////////////////////////////////////////////////////////////////////////////////////////
/*                        GENERAL INFORMATION
///////////////////////////////////////////////////////////////////////////////////////////////////

Pins for More or less controlling arduino:
A4 (I2C SDA)
A5 (I2C SCL)


Motor: (for ball grabbing)
2 Pins, analog (?)

Light Sensor:
1 Pin, digital

Ultrasonic:
2 Pins:



Other required:
3 or more Light-Sensors, arranged next to each other in certain distances.
Easily changeable (hopefully).


Specs:
Arduino Nano
Connected with:
1 Motor for taking up the ball
1 Ultrasonic Sensor
Several light-sensitive Sensors, for following a line
Connected on an I2C bus, will return current should-be
direction if asked for.


Optional (might not be included in code as of yet):
Light sensors put up on front, detecting the line several
centimeters before the actual vehicle.


*/ // end of general information
///////////////////////////////////////////////////////////////////////////////////////////////////


// for ultrasonic sensor
int trigPin = 10; // some digital pin
int echoPin = 11; // some digital pin
int distance, old_distance, diff;
NewPing sonar(trigPin, echoPin, 20);


// for light sensors
int lsensor[3] = {7, 8, 9};    // left, middle, right sensor
bool light_d[3];


// ball sensor
int ballsensor = 12; // checking if the ball is there or not


// grabbing motor:
int pos = 0;
int newPos = 0;
int servoPin = 6;
Servo myServo;


// loop control
int STD_LOOP_TIME = 499; //499= 500us loop time // code that keeps loop time at 500us per cycle of main program loop
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;


// own arduino state
int own_state = 0;
bool askForDist = false;
bool readLight = false;
bool isBall = false;
bool haveBall = false;


// other arduino state
int other_state = 0;
int percent = 0;
int direction = 0;
uint8_t others_old = 0;  // other eight sensors (old values)
uint8_t others = 0;  // other eight sensors


///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if (DEBUG)
    Serial.begin(9600);

  Wire.begin(8); // Join I2C bus with slave address #8
  TWBR = 24; // 400kHz I2C clock (Fast)

  myServo.attach(servoPin); // attaches the servo pin, for controlloing over the object now.

  Wire.onRequest(return_values);
  Wire.onReceive(recv_event);

  pinMode(ballsensor, INPUT);

  pinMode(lsensor[0], INPUT);
  pinMode(lsensor[1], INPUT);
  pinMode(lsensor[2], INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  pinMode(LED_BUILTIN, OUTPUT);

  sonar.ping_cm(); // initializing sonar.

} // end of setup


///////////////////////////////////////////////////////////////////////////////////////////////////
void return_values() {
  /////////////////////////////////////////////////////////////////////////////////////////////////
  if (own_state == 0) {
    Wire.write(1);
  } else if (own_state < 8) {
    Wire.write(own_state);
    if (askForDist)
      Wire.write(distance);
    if (readLight) {
      Wire.write(2 * light_d[0] | 4 * light_d[1] | 8 * light_d[2]);
    }
    if (own_state == 3 || own_state == 7) {
      Wire.write(haveBall);
    }
  }

  if (DEBUG) {
    Serial.print("state: ");
    Serial.print(own_state);
    if (askForDist) {
      Serial.print(" dist: ");
      Serial.print(distance);
    }
    Serial.println("");
  } // end of DEBUG
} // end of return_values


///////////////////////////////////////////////////////////////////////////////////////////////////
void recv_event(int num_bytes) {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  other_state = Wire.read();

  if (other_state != 4) {
    others_old = others;
    others = Wire.read();
  }

  if (own_state != other_state) {
    // TODO: change own state somehow.
  }

} // end of recv_event



///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {         // Main Control loop
  /////////////////////////////////////////////////////////////////////////////////////////////////

  lastLoopUsefulTime = micros() - loopStartTime;

  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    delayMicroseconds(STD_LOOP_TIME - lastLoopUsefulTime);
  }

  lastLoopTime = micros() - loopStartTime;
  loopStartTime = micros();

  read_sensors();

  determine_action();
} // end of loop



///////////////////////////////////////////////////////////////////////////////////////////////////
void read_sensors() {         // Reading the ultrasonic- and light sensors
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // Ultrasonic reading happens here
  if (askForDist) {
    old_distance = distance;    // old value for more accuracy - current vaule might not be exact
    distance = sonar.ping_cm(); // getting new current distance
    if (distance != 0 && old_distance != 0) {
      diff = abs(distance - old_distance);
    }
  } // end of askForDist

  isBall = digitalRead(ballsensor);

  if (readLight) {
    for (int i = 0; i < 3; i++)
      light_d[i] = digitalRead(lsensor[i]);
  }

} // end of read_sensors




///////////////////////////////////////////////////////////////////////////////////////////////////
void set_state(int newstate) {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  switch(own_state) {

    case 0:   // startup and initializion of sensors
      break;

    case 1:   // startup complete, follow line, listen for ultrasonic
    case 5:   // follow line, listen for ultrasonic
      askForDist = true;
      readLight = false;
      break;

    case 2:   // closing in on box
    case 6:   // closing in on box again
      askForDist = true;
      readLight = true;
      break;

    case 3:   // found box, grab ball
      askForDist = false;
      readLight = false;
      newPos = 36;
      break;

    case 4:   // grabbed ball, turn (This Arduino does nothing)
      askForDist = false;
      readLight = false;
      haveBall = true;
      break;

    case 7:   // found second box, drop ball
      askForDist = false;
      readLight = false;
      newPos = 150;
      break;

    case 8:   // finished.
      askForDist = false;
      readLight = false;
      newPos = 36;
      haveBall = false;
      break;

    case 9:   // Error somewhere. FAIL.
      askForDist = false;
      readLight = false;
      break;

    default:  // shouldn't happen
      askForDist = false;
      readLight = false;
      digitalWrite(LED_BUILTIN, HIGH);
      break;

  } // end of state-switch
} // end of set_state



///////////////////////////////////////////////////////////////////////////////////////////////////
void determine_action() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: rewrite

  /*
  // As it turns out, all this arduino does is reading it's sensors and sending the values
  // if requested ... so nothing in particular
  switch(own_state) {

    case 0:   // startup and initializion of sensors
      break;

    case 1:   // startup complete, follow line, listen for ultrasonic
    case 5:   // follow line, listen for ultrasonic
      break;

    case 2:   // closing in on box
    case 6:   // closing in on box again
      break;

    case 3:   // found box, grab ball
      break;

    case 4:   // grabbed ball, turn (This Arduino does nothing)
      break;

    case 7:   // found second box, drop ball
      break;

    case 8:   // finished.
      break;

    case 9:   // Error somewhere. FAIL.
      break;

    default:  // shouldn't happen
      delay(500);
      break;

  } // end of state-switch

  */

  if ((own_state == 3 || own_state == 7) && newPos == pos) {
    set_state(own_state + 1);
  }

  if (newPos > pos)
    pos++;
  if (newPos < pos)
    pos--;

} // end of determine_action




///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Ultrasonic control section                    //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

/* so much for that ... we simly read out the distance only after all

///////////////////////////////////////////////////////////////////////////////////////////////////
void close_in() {             // closing in on the box
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if ((distance < 3 && distance != 0) || (old_distance < 3 && old_distance != 0)) {
    set_state(own_state + 1);
  } else if (distance != 0 || old_distance != 0) {
    // direction = 0;
    percent += 4;
  }

} // end of close_in



///////////////////////////////////////////////////////////////////////////////////////////////////
void stay_in_dist() {         // staying as close to the box as we already are
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if ((distance < 3 && distance != 0) || (old_distance < 3 && old_distance != 0)) {
    // direction = 0;
    percent -= 2;
  } else if (distance != 0 || old_distance != 0) {
    // direction = 0;
    percent += 2;
  }
} // end of stay_in_dist

*/





///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Ball ... control section                      //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

/* happening somewhere else already ... easier than expected


///////////////////////////////////////////////////////////////////////////////////////////////////
void grab_ball() {        // actually Grabbing the ball. verifying with a sensor?
  /////////////////////////////////////////////////////////////////////////////////////////////////
  newPos = 135;
} // end of grab_ball



///////////////////////////////////////////////////////////////////////////////////////////////////
void release_ball() {        // actually Grabbing the ball. verifying with a sensor?
  /////////////////////////////////////////////////////////////////////////////////////////////////
  newPos = 36;
} // end of grab_ball

*/




/*
  Example Code found at: http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/?ALLSTEPS
*/
