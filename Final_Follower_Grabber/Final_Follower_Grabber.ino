
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


int stage = 0;


// for ultrasonic sensor
int trigPin = 10; // some digital pin
int echoPin = 11; // some digital pin
int distance, old_distance, diff;
NewPing sonar(trigPin, echoPin, 14);
bool askForDist = false;


// for light sensors
int lsensor[3] = {7, 8, 9};    // left, middle, right sensor
int light_d[3];
uint8_t others_old = 0;  // other eight sensors (old values)
uint8_t others = 0;  // other eight sensors
bool readLight = false;


// ball sensor
int ballsensor = 12; // checking if the ball is there or not
bool isBall = false;


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



// other arduino state
int finished = 0;
int percent = 0;
int direction = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  /////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(8); // Join I2C bus with slave address #8
  TWBR = 24; // 400kHz I2C clock (Fast)
  myServo.attach(servoPin); // attaches the servo pin, for controlloing over the object now.

  Wire.onRequest(return_values);
  Wire.onReceive(recv_event);

  pinMode(lsensor[0], INPUT);
  pinMode(lsensor[1], INPUT);
  pinMode(lsensor[2], INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

} // end of setup


///////////////////////////////////////////////////////////////////////////////////////////////////
void return_values() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

} // end of return_values


///////////////////////////////////////////////////////////////////////////////////////////////////
void recv_event(int num_bytes) {
  /////////////////////////////////////////////////////////////////////////////////////////////////

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

  // determine_action();

  if (newPos > pos)
    pos++;
  if (newPos < pos)
    pos--;

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
void determine_action() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: rewrite

  switch(stage) {

    case 1:   // startup complete, search for box
      close_in();
      break;

    case 2:   // found box, grab ball
      stay_in_dist();
      grab_ball();
      break;

    case 3:   // grabbed ball, turn
      // TODO: Turning. Includes: reverse motor, turn right, reverse again, turn right again. hardcoded?
      break;

    case 4:   // follow line
      break;

    case 5:   // near end
      // TODO: Trigger? Timer?
      close_in();
      break;

    case 6:   // found second box, drop ball
      stay_in_dist();
      release_ball();
      break;

    case 7:   // finished.
      send_finish(1);
      break;

    case 8:   // fell over. FAIL.
      send_finish(2);
      break;

    default:  // shouldn't happen
      send_finish(3);
      break;

    } // end of stage-switch
} // end of determine_action



///////////////////////////////////////////////////////////////////////////////////////////////////
void set_stage(int newstage) {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  switch(stage) {

    case 0:   // startup and initializion of sensors
      break;

    case 1:   // startup complete, search for box
      break;

    case 2:   // found box, grab ball
      break;

    case 3:   // grabbed ball, turn
      break;

    case 4:   // follow line
      break;

    case 5:   // near end
      break;

    case 6:   // found second box, drop ball
      break;

    case 7:   // finished.
      break;

    case 8:   // fell over. FAIL.
      break;

    default:  // shouldn't happen
      askForDist = false;
      readLight = false;
      break;

    } // end of stage-switch
} // end of set_stage





///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Ultrasonic control section                    //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
void close_in() {             // closing in on the box
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if ((distance < 3 && distance != 0) || (old_distance < 3 && old_distance != 0)) {
    set_stage(stage + 1);
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






///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Ball ... control section                      //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


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






///////////////////////////////////////////////////////////////////////////////////////////////////
void send_finish(int8_t message) {      // finished with task, end.
  /////////////////////////////////////////////////////////////////////////////////////////////////
  finished = message;

}

/*
  Example Code found at: http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/?ALLSTEPS
*/
