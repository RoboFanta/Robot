
/* What to do

TODO:
* merge
  - ultrasonic
  - light sensor reading
  - line following
* write missing:
  - motor control
  - grabbing
  - turning
  - other

next steps:
> working framework .

*/



// One Arduino - Final Code for following a line for only one Arduino after all.
// not using a Gyro or 11 light sensors, only 8 (+1) and an ultrasonic.


// Last Modified:
// 2017-02-14     For Uni Project.     Felix Karg


/*
This Arduino code will take inputs from <n> sensors
and tries to complete the task of taking up a ball,
following a line, and dropping the ball at the designated point.


Input:
1 Light sensor for sensing the ball.
1 Ultrasonic sensor.

Output:
1 Motor for grabbing a ball.

Written mainly by   Felix Karg     <felix.karg@uranus.uni-freiburg.de>
with changes from   Victor Maier   <>
              and   Paul Boeninger <>

For trying a Arduino-style robot at the SDP Project, University of Freiburg, WS2016/17.

License is GPL-v3.
(included in subdirectory)
*/



#include <NewPing.h>
#include <Servo.h>

#define DEBUG 1
// #define DEBUG 0


///////////////////////////////////////////////////////////////////////////////////////////////////
/*                        GENERAL INFORMATION       (for the Arduino)
///////////////////////////////////////////////////////////////////////////////////////////////////

Pins for More or less controlling arduino:

2 x Motor: (for driving)
2 Pins, 5 + 9, 6 + 10

8 x Light Sensor:
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

(Generalized pin layout is included as nano.pdf)

*/ // end of general information
///////////////////////////////////////////////////////////////////////////////////////////////////


// Defining some stuff
// from victors code

typedef struct {int direction; int speed;} motorstate;
typedef int (*read_pins)(uint8_t pin);
typedef void (*void_ptr)();


//array längen
#define lmain_length 5
#define lfront_length 3

#define SPEED_VSLOW   48
#define SPEED_SLOW    92
#define SPEED_MIDDLE  128
#define SPEED_FAST    178
#define SPEED_VFAST   255

#define DIRECTION_RIGHT_3 127
#define DIRECTION_RIGHT_2 63
#define DIRECTION_RIGHT_1 31
#define DIRECTION_NON     0
#define DIRECTION_LEFT_1 -31
#define DIRECTION_LEFT_2 -63
#define DIRECTION_LEFT_3 -127
#define oldvar_size (1<<4)

#define STD_LOOP_TIME 49
//49= 50us loop time

#define ERROR_TIME ((STD_LOOP_TIME+1)*400)
#define MOTOR_STOP 0
#define MOTOR_MAX 255

// Servo winkel
#define SERVO_OPEN 138
#define SERVO_CLOSE 43

//Scan heufigkeit 1/scan_freq
#define SCANS_FREQ 10

//Pinconfig
// Motor Right:
#define motor_right_forward 5 
#define motor_right_reverse 10

// Motor Left:
#define motor_left_forward 6
#define motor_left_reverse 11

// pins
#define SCHALTER_PIN 19
#define SERVO 9
#define US_TRIGGER 12
#define US_ECHO 2
#define TRIGPIN 10
#define ECHOPIN 11




//states array with function pointers
const void_ptr states[] = {init_prog, start_prog, line_way1, get_ball, wenden, line_way2, unget_ball, &finish};

//aktuell ausgeführt
void_ptr current_action = &init_prog;



// for ultrasonic sensor
int distance, old_distance, diff;
NewPing sonar(TRIGPIN, ECHOPIN, 20); // 20 cm of maximum distance we wait for the return


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


//5er reihe sensoren
const int lmain[lmain_length]={12,14,15,16,17};
//3er reihe lichtsensoren vorne
const int lfront[lfront_length]={4,7,8};
//array alte werte
uint8_t oldvar_ptr=0;
uint8_t oldvars[1<<4];
bool readLight = false;

// motor control
int percent = 0;
int direction = 0;

// loop control
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

// own arduino
motorstate moto;
bool askForDist = false;
bool isBall = false;


///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {          // initializing happens here
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if (DEBUG)
    Serial.begin(9600);

  myServo.attach(servoPin); // attaches the servo pin, for controlloing over the object now.

  pinMode(ballsensor, INPUT);


  // setting the five main light sensors as input pins
  for(int i=0; i < lmain_length; i++)
    pinMode(lmain[i], INPUT);

  // setting the three front light sensors as input pins
  for(int i=0; i < lfront_length; i++)
    pinMode(lfront[i], INPUT);

  pinMode(motor_right_forward, OUTPUT);
  pinMode(motor_right_reverse, OUTPUT);

  pinMode(motor_left_forward, OUTPUT);
  pinMode(motor_left_reverse, OUTPUT);


  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  sonar.ping_cm(); // initializing sonar.

} // end of setup



///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {         // Main Control loop
  /////////////////////////////////////////////////////////////////////////////////////////////////

  lastLoopUsefulTime = micros() - loopStartTime;

  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    delayMicroseconds(STD_LOOP_TIME - lastLoopUsefulTime); // code that keeps loop time at 50us per cycle of main program loop
  }

  lastLoopTime = micros() - loopStartTime;
  loopStartTime = micros();

  read_sensors();

  current_action(); // TODO: use @Victors method.

  set_motors();

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




uint8_t read_X_pins (int* pins, uint8_t lenght){
  uint8_t x = 0;
  while(lenght--){
    x |= digitalRead(pins[lenght]) << lenght;
  }
  return x;
}




///////////////////////////////////////////////////////////////////////////////////////////////////
void follow_line() {    // line-following logic is going to happen here !
  /////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t tmp=0;

  oldvars[oldvar_ptr =
    (oldvar_ptr + 1) & (oldvar_size-1)] =
      read_X_pins(lmain, lmain_length) /*|(read_X_pins(lmain, lmain_length)<<5)*/;
  switch(tmp&0x18){ //die aeuseren
    case 0x18://00
    //moto.speed = /*const*/;
    break;

    case 0x10://01
    moto.speed = SPEED_VSLOW;
    direction = DIRECTION_RIGHT_3;
    return moto;

    case 0x08://10
    moto.speed = SPEED_VSLOW;
    direction = DIRECTION_LEFT_3;
    return moto;

    case 0x00://11
    error();
    direction = DIRECTION_NON;
    break;
  }
  switch(tmp&0x07){
    case 0x00: //000
    //use old vars to find line
    break;

    case 0x01: //001
    moto.speed = SPEED_MIDDLE;
    direction = DIRECTION_RIGHT_2;
    return moto;

    case 0x02: //010
    error();
    break;

    case 0x03: //011
    moto.speed = SPEED_FAST;
    direction = DIRECTION_RIGHT_1;
    return moto;

    case 0x04: //100
    moto.speed = SPEED_MIDDLE;
    direction = DIRECTION_LEFT_2;
    return moto;

    case 0x05: //101
    error();
    break;

    case 0x06: //110
    moto.speed = SPEED_FAST;
    direction = DIRECTION_LEFT_1;
    return moto;

    case 0x07: //111
    //On the way
    break;
  }
} // end of follow_line





///////////////////////////////////////////////////////////////////////////////////////////////////
void set_motors() {
  /////////////////////////////////////////////////////////////////////////////////////////////////


} // end of set_motors

/*
//geht nicht rückwärz
void set_motors(motorstate moto) {
  if(moto.direction!=DIRECTION_NON){
    if(moto.direction<DIRECTION_NON){
      analogWrite(motor_left_forward,((int)(moto.speed)+(int)(moto.direction)<MOTOR_STOP)? MOTOR_STOP : (moto.speed+moto.direction) );
      analogWrite(motor_right_forward,((int)(moto.speed)-(int)(moto.direction)>MOTOR_MAX)? MOTOR_MAX : (moto.speed-moto.direction) );
      analogWrite(motor_left_reverse,((int)(moto.speed)+(int)(moto.direction)< MOTOR_STOP)?  -(moto.speed+moto.direction) : MOTOR_STOP) ;
      analogWrite(motor_right_reverse,MOTOR_STOP);
    }
    else{
      analogWrite(motor_left_forward,((int)(moto.speed)+(int)(moto.direction)>MOTOR_MAX)?MOTOR_MAX:moto.speed+moto.direction);
      analogWrite(motor_right_forward,((int)(moto.speed)-(int)(moto.direction)<MOTOR_STOP)?MOTOR_STOP:moto.speed-moto.direction);
      analogWrite(motor_left_reverse,MOTOR_STOP);
      analogWrite(motor_right_reverse,((int)(moto.speed)-(int)(moto.direction) < MOTOR_STOP) ? -(moto.speed-moto.direction) : MOTOR_STOP);
    }
  }
  else{
      analogWrite(motor_right_forward,moto.speed);
      analogWrite(motor_left_forward,moto.speed);
      analogWrite(motor_right_reverse,MOTOR_STOP);
      analogWrite(motor_left_reverse,MOTOR_STOP);
  }
} // end of set_motors
*/


///////////////////////////////////////////////////////////////////////////////////////////////////
void close_in() {             // closing in on the box
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // TODO: get closer to the box using the ultrasonic information

} // end of close_in





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
  Example Code for Ultrasonic found at: 
  http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/?ALLSTEPS
*/








void error(){
  digitalWrite(LED_BUILTIN,HIGH);
  delay(500);
}














void init_prog(){
  distance = sonar.ping_cm();
  current_state=states[1];
}

void start_prog(){
}
void line_way1(){
  current_state=states[3];
}
void get_ball(){
  current_state=states[3];
}
void wenden(){
  current_state=states[3];
}
void line_way2(){
  current_state=states[3];
}
void unget_ball(){
  current_state=states[3];
}
void finish(){
  
}


