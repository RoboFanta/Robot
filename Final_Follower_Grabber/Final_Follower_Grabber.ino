
// Final Follower - Final Code for following a line for up to 2 Arduinos,
// One of which may be controlling two motors based on a Gyro.


// Last Modified:
// 2017-02-06     For Uni Project.     Felix Karg


/*
This Arduino code will take inputs from a few light- and an ultrasonic sensor,
and communicates with a second arduino in taking up a ball, navigating,
as in following a line, and dropping the ball at the designated point.


Written mainly by   Felix Karg     <felix.karg@uranus.uni-freiburg.de>
improvements from   Paul Boeninger <>
              and   Victor Maier   <>

For trying a Segway-style robot at the SDP Project, University of Freiburg, WS2016/17.

Only Files including this line are Licensed under CC 2.5
(Available here: https://creativecommons.org/licenses/by/2.5/legalcode )
*/



#include <Wire.h>
#include "I2Cdev.h"
#include <NewPing.h>


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
Up to 3 Pins ?


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
long duration, distance, old_distance;
NewPing sonar(trigPin, echoPin, 14);
bool askForDist = false;


// for light sensors
int lsensor1 = 7;    // left sensor
int lsensor2 = 8;    // middle sensor
int lsensor3 = 9;    // right sensor
uint8_t lsens[3];    // [l1, l2, l3] - Values
uint8_t olsens[3];    // [l1, l2, l3] - old Values
// uint8_t elsens[3];   // [l1, l2, l3] - expected Values
bool readLight = false;

// motor 'state'
int tmp;
int8_t finished = 0;
int8_t percent = 0;
int8_t direction = 0;
/*  -3 = LEEFT!!
    -2 = more left
    -1 = slight left
     0 = straight ahead
     1 = slight right
     2 = more right
     3 = To the RIIIGHT!!
*/

// grabbing motor:
int grabPin1 = 5;
int grabPin2 = 6;

/* // Only when Motor control is going happen on this Arduino after all.
// For controlling the Motor
#define OnFwd(pin,speed) MotorUse(pin,speed*256.0/100.0, false)
#define OnRev(pin,speed) MotorUse(pin,speed*256.0/100.0, true)
#define Off(pin) MotorUse(pin, 0, false)

// Defining the Control pins for the motors.
#define OUT_A motor1forward
#define OUT_B motor2forward
int motor1forward = 5;
int motor2forward = 6;
int motor1reverse = 10;
int motor2reverse = 11;
*/


int STD_LOOP_TIME = 49; //49= 50us loop time // code that keeps loop time at 50us per cycle of main program loop
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  /////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(8); // Join I2C comm with slave address #8
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  Wire.onRequest(return_values);
  Wire.onReceive(recv_event);

  pinMode(lsensor1, INPUT);
  pinMode(lsensor2, INPUT);
  pinMode(lsensor3, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

} // end of setup


///////////////////////////////////////////////////////////////////////////////////////////////////
void return_values() {
  /////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.write(direction);
  Wire.write(percent);
  Wire.write(finished);
}

void recv_event(int num_bytes) {
  int new_state = Wire.read();
  set_stage(new_state);
}



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

  direction = 0;
  percent = 0;

  determine_direction();

} // end of loop



///////////////////////////////////////////////////////////////////////////////////////////////////
void read_sensors() {         // Reading the ultrasonic- and light sensors
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // reading light sensors happens here
  if (readLight) {
    // setting the old values
    olsens[0] = lsens[0];
    olsens[1] = lsens[1];
    olsens[2] = lsens[2];

    // Light Sensor reading goes here
    lsens[0] = digitalRead(lsensor1);
    lsens[1] = digitalRead(lsensor2);
    lsens[2] = digitalRead(lsensor3);
  } // end of readLight

  // Ultrasonic reading happens here
  if (askForDist) {
  /* Actually, that's just old.
//  digitalWrite(trigPin, LOW);   // actually required?
//  delayMicroseconds(2);         // actually required?
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);          // requesting a measurement
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 400); // reading time until it came back, maximum of 13.2 cm
    old_distance = distance;
    distance = (duration/2) / 29.1;   // calculating the distance from it
    */
    old_distance = distance;    // old value for more accuracy - current vaule might not be exact
    distance = sonar.ping_cm(); // getting new current distance
    if (distance != 0 && old_distance != 0) {
      diff = abs(distance - old_distance);
    }
  } // end of askForDist

} // end of read_sensors



///////////////////////////////////////////////////////////////////////////////////////////////////
void determine_direction() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  switch(stage) {

    case 1:   // startup complete, search for box
      follow_line();
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
      follow_line();
      break;

    case 5:   // near end
      // TODO: Trigger? Timer?
      follow_line();
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
} // end of determine_direction



///////////////////////////////////////////////////////////////////////////////////////////////////
void set_stage(int newstage) {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  switch(stage) {

    case 0:   // startup and initializion of sensors
      askForDist = false;
      readLight = false;
      break;

    case 1:   // startup complete, search for box
      askForDist = true;
      readLight = true;
      break;

    case 2:   // found box, grab ball
      askForDist = true;
      readLight = false;
      break;

    case 3:   // grabbed ball, turn
      askForDist = false;
      readLight = false;
      break;

    case 4:   // follow line
      askForDist = false;
      readLight = true;
      break;

    case 5:   // near end
      askForDist = true;
      readLight = true;
      break;

    case 6:   // found second box, drop ball
      askForDist = true;
      readLight = false;
      break;

    case 7:   // finished.
      askForDist = false;
      readLight = false;
      break;

    case 8:   // fell over. FAIL.
      askForDist = false;
      readLight = false;
      break;

    default:  // shouldn't happen
      askForDist = false;
      readLight = false;
      break;

    } // end of stage-switch
} // end of set_stage


///////////////////////////////////////////////////////////////////////////////////////////////////
void follow_line() {    // line-following logic is going to happen here !
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // TODO: @Victor this needs improvement.

  tmp = 0;

  if (lsens[0]) {
    tmp |= 1;
  }
  if (lsens[1]) {
    tmp |= 2;
  }
  if (lsens[2]) {
    tmp |= 4;
  }

  switch(tmp){
    case 4: //hard right
      direction += 3;
      percent += 10;
      break;

    case 1: //hard left
      direction += -3;
      percent += 10;
      break;

    case 6: //light left
      direction += -2;
      percent += 20;
      break;

    case 3: //light right
      direction += 2;
      percent += 20;
      break;

    case 2: /*nothing :D*/
      // direction = 0;
      percent += 30;
      break;

//    case 0: //all 0
//    case 5: //middle nothing
//    case 7: //all 1
    default:
      delay(500); // might be deadly. Who knows
      // direction = 0;
      // percent = 0;
      break;

  } // end of tmp-switch
} // end of follow_line





///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Ultrasonic control section                    //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: Paul - improve!


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
// TODO: Paul - improve!


///////////////////////////////////////////////////////////////////////////////////////////////////
void grab_ball() {        // actually Grabbing the ball. verifying with a sensor?
  /////////////////////////////////////////////////////////////////////////////////////////////////
  analogWrite(grabPin1, 50);
  delay(500);          // this might need improvement, might not stay in correct distance otherwise
  analogWrite(grabPin1, 0);
} // end of grab_ball



///////////////////////////////////////////////////////////////////////////////////////////////////
void release_ball() {        // actually Grabbing the ball. verifying with a sensor?
  /////////////////////////////////////////////////////////////////////////////////////////////////
  analogWrite(grabPin2, 50);
  delay(500);          // this might need improvement, might not stay in correct distance otherwise
  analogWrite(grabPin2, 0);
} // end of grab_ball






///////////////////////////////////////////////////////////////////////////////////////////////////
void send_finish(int8_t message) {      // finished with task, end.
  /////////////////////////////////////////////////////////////////////////////////////////////////
  finished = message;

}



/* // Only when Motor control is going happen on this Arduino after all.
void MotorUse(int pin, int speed, bool reverse){
  analogWrite(pin + 5, speed * reverse);
  analogWrite(pin, speed * (not reverse));
}
*/


/*
  Example Code found at: http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/?ALLSTEPS
*/
