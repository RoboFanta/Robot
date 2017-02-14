
// Final Follower - Final Code for following a line for up to 2 Arduinos,
// One of which may be controlling two motors based on a Gyro.


// Last Modified:
// 2017-02-06     For Uni Project.     Felix Karg


/*
This Arduino code will take inputs from a few lightsensors,
and communicates with a second arduino in completing the task of taking up a ball,
following a line, and dropping the ball at the designated point.


Input:
11 Light sensors.

Output:
2 Motors

Communication:
I2C Master, Second Arduino having one ball sensor, ball motor and ultrasonic sensor.


Written mainly by   Felix Karg     <felix.karg@uranus.uni-freiburg.de>
improvements from   Paul Boeninger <>
              and   Victor Maier   <>

For trying a Arduino-style robot at the SDP Project, University of Freiburg, WS2016/17.

License is GPL-v3.
(included in subdirectory)
*/





#include <Wire.h>
#include <I2Cdev.h>







///////////////////////////////////////////////////////////////////////////////////////////////////
/*                        GENERAL INFORMATION
///////////////////////////////////////////////////////////////////////////////////////////////////

Pins for More or less controlling arduino:
A4 (I2C SDA)
A5 (I2C SCL)


2 Motors:
2 Pins each, both pwm.
Pins:
5, 10
6, 11


Light Sensors:
1 Pin, digital
Pins:
4, 7, 8, 12,
14, 15, 16, 17

Lightsensor-Array:

  x x x     <- front row, probably inaccurate
  N_cm_
 [xxxxx]    <- main row, following line
 |10 cm|    <- Robot
 [x_x_x]    <- correcting row, other arduino


Specs:
Arduino Nano
Connected with:
2 Motors for driving
8 light sensors for primary navigation


Optional (might not be included in code as of yet):
Light sensors put up on front, detecting the line several
centimeters before the actual vehicle.


*/ // end of general information
///////////////////////////////////////////////////////////////////////////////////////////////////

//5er reihe sensoren
#define lmain_lenght 5
#define lfront_lenght 3
const int lmain[lmain_lenght]={12,14,15,16,17};
//3er reihe lichtsensoren vorne
const int lfront[lfront_lenght]={4,7,8};
//array alte werte
uint8_t oldvar_ptr=0;
uint8_t oldvars[1<<4];
bool readLight = false;

// motor control
int direction = 0;
/*
    -3 = LEEFT!!
    -2 = more left
    -1 = slight left
     0 = straight ahead
     1 = slight right
     2 = more right
     3 = To the RIIIGHT!!
*/


// Motor Right:
int mrforward = 5;
int mrreverse = 10;

// Motor Left:
int mlforward = 6;
int mlreverse = 11;


// own Arduino
int own_stage = 0;
bool askForDist = false;
bool askForLightOther = false;


// other Arduino:
int other_stage = 0;
int distance = 0;
uint8_t lightOther = 0;


// loop control
int STD_LOOP_TIME = 49; //49= 50us loop time // code that keeps loop time at 50us per cycle of main program loop
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  /////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(); // Join I2C bus as master.
  TWBR = 24; // 400kHz I2C clock (Fast)

  Wire.onRequest(return_values);
  Wire.onReceive(recv_event);

  // setting the five main light sensors as input pins
  for(int i=0; i < lmain_lenght; i++)
    pinMode(lmain[i], INPUT);

  // setting the three front light sensors as input pins
  for(int i=0; i < lfront_lenght; i++)
    pinMode(lfront[i], INPUT);

  pinMode(mrforward, OUTPUT);
  pinMode(mrreverse, OUTPUT);

  pinMode(mlforward, OUTPUT);
  pinMode(mlreverse, OUTPUT);

} // end of setup



/* not happening, as this is master

///////////////////////////////////////////////////////////////////////////////////////////////////
void return_values() {
  /////////////////////////////////////////////////////////////////////////////////////////////////


} // end of return_values


///////////////////////////////////////////////////////////////////////////////////////////////////
void recv_event(int num_bytes) {
  /////////////////////////////////////////////////////////////////////////////////////////////////


} // end of recv_event

*/



///////////////////////////////////////////////////////////////////////////////////////////////////
void request_values() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  Wire.requestFrom(8, n);
  Wire.read();


} // end of request_values



///////////////////////////////////////////////////////////////////////////////////////////////////
void send_values() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  Wire.beginTransmission(8);
  Wire.write(own_stage);
  Wire.endTransmission();

} // end of send_values



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

  follow_line();

  set_motors();

} // end of loop



uint8_t read_X_pins (int* pins, uint8_t lenght){
  uint8_t x = 0;
  while(lenght--){
    x |= digitalRead(pins[lenght]) << lenght;
  }
  return x;
}




///////////////////////////////////////////////////////////////////////////////////////////////////
void read_sensors() {         // Reading the ultrasonic- and light sensors
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // light sensor reading goes here
  if (readLight) {
  }

} // end of read_sensors




///////////////////////////////////////////////////////////////////////////////////////////////////
void follow_line() {    // line-following logic is going to happen here !
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // @Victor: TODO: write

  // should only set the variable direction
  /*  -3 = LEEFT!!
      -2 = more left
      -1 = slight left
       0 = straight ahead
       1 = slight right
       2 = more right
       3 = To the RIIIGHT!!
  */

} // end of follow_line




///////////////////////////////////////////////////////////////////////////////////////////////////
void set_motors() {
  /////////////////////////////////////////////////////////////////////////////////////////////////


} // end of set_motors



///////////////////////////////////////////////////////////////////////////////////////////////////
void determine_action() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: rewrite

  switch(own_stage) {

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
} // end of determine_action



///////////////////////////////////////////////////////////////////////////////////////////////////
void set_stage(int newstage) {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  switch(newstage) {

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
} // end of set_stage



///////////////////////////////////////////////////////////////////////////////////////////////////
void close_in() {             // closing in on the box
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // TODO: get closer to the box using the ultrasonic information

} // end of close_in


