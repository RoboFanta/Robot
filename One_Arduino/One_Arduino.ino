
/* What to do

TODO:
* merge
  - ultrasonic            (done)
  - light sensor reading  (probably finished)
  - line following        (in progress)
* write missing:
  - motor control         (done)
  - grabbing              (mostly done)
  - turning               (need to test)
  - closing in            (a LOT required)
  - other
* finish
  - ball control section  (in progress)
  - framework             (in progress)
  - this code             (in progress)
  - documentation         (oh darn ...)

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
#include <ervo.h>

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
2 Motors for driving
1 Motor (Servo) for taking up the ball
1 Ultrasonic Sensor
1 Ball sensor
8 Light Sensors


Optional (might not be included in code as of yet):
Light sensors put up on front, detecting the line several
centimeters before the actual vehicle.

(Generalized pin layout is included as nano.pdf)

*/ // end of general information
///////////////////////////////////////////////////////////////////////////////////////////////////


// Defining some stuff
                                // TODO: cleanup
// from victors code

typedef int (*read_pins)(uint8_t pin);
typedef void (*void_ptr)();

// For controlling the Motor
#define OnFwd(pin, count) MotorUse(pin, count, false)
#define OnRev(pin, count) MotorUse(pin, count, true)
#define Off(pin) MotorUse(pin, 0, false)

//array längen
#define lmain_length 5
#define lback_length 3

#define SPEED_VSLOW   30
#define SPEED_SLOW    65
#define SPEED_MIDDLE  120
#define SPEED_FAST    160
#define SPEED_VFAST   200

#define DIRECTION_RIGHT_3 127
#define DIRECTION_RIGHT_2 63
#define DIRECTION_RIGHT_1 31
#define DIRECTION_NON     0
#define DIRECTION_LEFT_1 -31
#define DIRECTION_LEFT_2 -63
#define DIRECTION_LEFT_3 -127

#define STD_LOOP_TIME 49
//49=> 50us loop time

#define ERROR_TIME ((STD_LOOP_TIME+1)*40)
#define MOTOR_STOP 0
#define MOTOR_MAX 255

// Servo winkel
#define SERVO_OPEN 143
#define SERVO_CLOSE 43

//Scan heufigkeit 1/scan_freq
#define SCANS_FREQ 10

//Pinconfig
// Motor Right:
#define OUT_R 5
#define motor_right_forward 5
#define motor_right_reverse 9

// Motor Left:
#define OUT_L 6
#define motor_left_forward 6
#define motor_left_reverse 10

// other
#define SCHALTER_PIN 19
#define SERVO 11
#define TRIGPIN 2
#define ECHOPIN 12



//states array with function pointers       1             2           3           4          5
const void_ptr states[] = {&init_prog, &follow_line, &open_hatch, &close_in, &close_hatch, &turn,
                                       &follow_line, &close_in, &open_hatch, &close_hatch, &finish};
//                             0            6             7           8           9          10


//aktuell ausgeführt
void_ptr current_action = states[0];



// for ultrasonic sensor
int distance, variance, old_distance = 25, diff;
NewPing sonar(TRIGPIN, ECHOPIN, 25);
// 20 cm of maximum distance -> up to 1200 us. We might get as far as ~5mm in that time.


// ball sensor
int ballsensor = 18; // checking if the ball is there or not

// key sensor
int keysensor = 19; // for having a key for the robot :)

// grabbing motor:
int pos = SERVO_CLOSE;
int newPos = pos;
Servo myServo;


//5er reihe sensoren
const int lmain[lmain_length]={4, 14, 7, 3, 8};
//3er reihe lichtsensoren hinten
const int lback[lback_length]={15, 16, 17};
bool light_m[5];
bool light_b[3];

// motor control
int speed = 0;
int direction = 0;

// loop control
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

// own arduino
int own_state = 0;
bool askForDist = false;
bool readLight = false;
bool readSecond = false;
bool isBall = false;
bool wasBall = false;
int iteration = 0;
int turnstate = 0;



// from paul

int maxspeed = 150; //Wert von 0 bis 255
int softturn = 40; //Wert von 0 bis 255
int hardturn = 40;   //Wert von 25 bis 255
float hardturnfaktor = 3;

int turnspeed = 150;

int approxspd = 150;
int approxsoftturn = 100; //

int ruecksetzspeed = 150;
int ruecksetzdelay = 2000; //Wie lange soll er nach dem Grabben zurückfahren (Speed = approxspd1)
int drehzyklus = 200; //Optimierung des Drehzyklus ab dem Momnt, in dem alle hinteren Sensoren "0" lesen

int molipl = 6; //ANALOG WRITE
int molimi = 10; //ANALOG WRITE
int morepl = 5; //ANALOG WRITE
int moremi = 9; //ANALOG WRITE             //MOTOREN


int vogali = 3;
int voli = 4;
int vomi = 14;
int vore = 7;
int vogare = 8;
int vogalival = 0;
int volival = 0;
int vomival = 0;
int voreval = 0;
int vogareval = 0;                              //VORDERE SENSORREIHE

bool hardturnlinks = false;
bool turnlinks = false;

float slow_factor = 1.0;





///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {          // initializing happens here
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if (DEBUG)
    Serial.begin(9600);

  myServo.attach(SERVO); // attaches the servo pin, for grabbing the ball later on.
  myServo.write(pos);

  pinMode(ballsensor, INPUT); // this sensor tells us if we actually have the ball or not
  pinMode(keysensor, INPUT);  // this sensor tells us if we actually want to start or not

  // setting the five main light sensors as input pins
  for(int i=0; i < lmain_length; i++)
    pinMode(lmain[i], INPUT);

  // setting the three front light sensors as input pins
  for(int i=0; i < lback_length; i++)
    pinMode(lback[i], INPUT);

  pinMode(motor_right_forward, OUTPUT);
  pinMode(motor_right_reverse, OUTPUT);

  pinMode(motor_left_forward, OUTPUT);
  pinMode(motor_left_reverse, OUTPUT);


  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

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

  speed = 0;
  direction = 0;

  read_sensors();

  current_action(); // TODO: use @Victors method.

  // set_motors();
  myServo.write(pos);

  ++iteration;

  if (DEBUG && iteration % 5 == 0)
    print_mesg();

} // end of loop



///////////////////////////////////////////////////////////////////////////////////////////////////
void read_sensors() {         // Reading the ultrasonic- and light sensors
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // Ultrasonic reading happens here
  if (iteration % 20 == 0 && askForDist) {
    // variance = distance;
    old_distance = distance ? distance : old_distance;    // latest accurate distance value
    // old_distance = distance && variance ? distance : old_distance;    // latest accurate distance value
    distance = sonar.ping_cm(); // getting new current distance
  } // end of ultrasonic reading

  isBall = digitalRead(ballsensor);

  if (readLight) {
    for (int i = 0; i < lmain_length; ++i)
      light_m[i] = digitalRead(lmain[i]);
  }
  if (readSecond) {
    for (int i = 0; i < lback_length; ++i)
      light_b[i] = digitalRead(lback[i]);
  }

  vogalival = digitalRead(vogali);                                    //lese Sensoren aus...
  volival = digitalRead(voli);
  vomival = digitalRead(vomi);
  voreval = digitalRead(vore);
  vogareval = digitalRead(vogare);                                      //...und speichere sie ab

/*
  if (iteration % 50 == 0) {

    digitalWrite(ustr,LOW);
    delayMicroseconds(2);
    digitalWrite(ustr,HIGH);
    delayMicroseconds(10);
    digitalWrite(ustr,LOW);
    usval = pulseIn(usec,HIGH);
    distance = (usval/2)/29;

  } */

} // end of read_sensors




uint8_t read_X_pins (int* pins, uint8_t lenght) {
  uint8_t x = 0;
  while(lenght--){
    x |= digitalRead(pins[lenght]) << lenght;
  }
  return x;
}



///////////////////////////////////////////////////////////////////////////////////////////////////
void follow_line_sub() {    // line-following logic is going to happen here !
  /////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t tmp = 0;

  for (int i = 0; i < lmain_length; ++i)
    tmp |= light_m[i]<<i;

  switch(tmp & 0x07){
    case 0x00: // 000
      geradeaus();
      break;

    case 0x01: // 001
      starkrechts();
      break;

    case 0x03: // 011
      leichtrechts();
      break;

    case 0x04: // 100
      starklinks();
      break;

    case 0x06: // 110
      leichtlinks();
      break;

    case 0x02: // 010
    case 0x05: // 101
      // ruecksetzen();
      error();
      break;

    case 0x07: // 111
      geradeaus();
      break;
  }


  switch(tmp & 0x18){ //die aeuseren
    case 0x00: // 00
      //speed = /*const*/;
      geradeaus();
      break;

    case 0x08: // 01
      starkrechts();
      break;

    case 0x10: // 10
      starklinks();
      break;

    case 0x18: // 11
      // ruecksetzen();
      error();
      // direction = DIRECTION_NON; // ?
      break;
  }

}


void follow_paul () {
  if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
    geradeaus();
  }
  else if ((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1)){
    geradeaus();
  }
  else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
    leichtrechts();
  }
  else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
    leichtlinks();
  }
  else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
          ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
          ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
          ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
    starkrechts();
    hardturnlinks = false;
  }
  else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
          ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
          ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
          ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
    starklinks();
    hardturnlinks = true;
  }
  else if ((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0)){
    if (hardturnlinks == true){
      starklinks();
    }
    else {
      starkrechts();
    }
  }
  else {
    stopp();
  }

}

/*

///////////////////////////////////////////////////////////////////////////////////////////////////
void follow_line_sub(bool slow = false) {    // line-following logic is going to happen here !
  /////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t tmp = 0;
  for (int i = 0; i < lmain_length; ++i)
    tmp |= (2 ^ i) * light_m[i];

  switch(tmp & 0x07){
    case 0x00: // 000
      speed = SPEED_FAST;
      direction = DIRECTION_NON; // continue driving forward
      break;

    case 0x01: // 001
      speed = SPEED_MIDDLE;
      direction = DIRECTION_RIGHT_2 / (slow + 1);
      break;

    case 0x03: // 011
      speed = SPEED_FAST;
      direction = DIRECTION_RIGHT_1 / (slow + 1);
      break;

    case 0x04: // 100
      speed = SPEED_MIDDLE;
      direction = DIRECTION_LEFT_2 / (slow + 1);
      break;

    case 0x06: // 110
      speed = SPEED_FAST;
      direction = DIRECTION_LEFT_1 / (slow + 1);
      break;

    case 0x02: // 010
    case 0x05: // 101
      error();
      break;

    case 0x07: // 111
      //On the way
      speed = SPEED_FAST;
      break;

  }


  switch(tmp & 0x18){ //die aeuseren
    case 0x00: // 00
      //speed = // const // ;
      break;

    case 0x08: // 01
      speed = SPEED_SLOW;
      direction = DIRECTION_RIGHT_2 / (slow + 1);

    case 0x10: // 10
      speed = SPEED_SLOW;
      direction = DIRECTION_LEFT_2 / (slow + 1);
      break;

    case 0x18: // 11
      error();
      // direction = DIRECTION_NON; // ?
      break;
  }

} // end of follow_line_sub

*/


///////////////////////////////////////////////////////////////////////////////////////////////////
void follow_line() {
  /////////////////////////////////////////////////////////////////////////////////////////////////
  readLight = true;
  askForDist = true;

  // follow_line_sub();
  follow_paul();

  if (old_distance <= 24) { // close in on box now.
    readLight = true;
    readSecond = true;
    askForDist = true;

    wasBall = isBall;

    current_action = states[++own_state];
  }
} // end of follow_line





///////////////////////////////////////////////////////////////////////////////////////////////////
void set_motors() {           // controlling direction happens here
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if (direction == DIRECTION_NON) {   // no direction
    OnFwd(OUT_R, speed);
    OnFwd(OUT_L, speed);
    // return;   // we do not need to check the other cases, return void.
  }

  if ( abs(direction) <= speed) {     // the change in direction is relatively small
    OnFwd(OUT_R, speed - direction);
    OnFwd(OUT_L, speed + direction);
  } else {                            // the change in direction is quite big
    int adir = abs(direction);
    direction < 0 ? OnFwd(OUT_R, (adir + speed) / 2 ) : OnRev(OUT_R, adir - speed);
    direction > 0 ? OnFwd(OUT_L, (adir + speed) / 2 ) : OnRev(OUT_L, adir - speed);
  }
} // end of set_motors




///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Ultrasonic control section                    //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

/*
  Example Code for Ultrasonic found at:
  http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/?ALLSTEPS
*/


///////////////////////////////////////////////////////////////////////////////////////////////////
void close_in() {             // closing in on the box
  /////////////////////////////////////////////////////////////////////////////////////////////////

  readLight = true;
  readSecond = true;
  askForDist = true;

  bool close_enough = false;

  // using line-following abstraction here.

  slow_factor = 0.5;

  // follow_line_sub();
  follow_paul();

  if (analogRead(A6) < 128)
    leichtrechts();
  if (analogRead(A7) < 128)
    leichtlinks();

  // speed = old_distance <= 6 ? SPEED_VSLOW : SPEED_SLOW;


  close_enough = (old_distance <= 4 && wasBall) || (isBall && !wasBall);

  if (close_enough) { // going to next stage ...
    readLight = true;
    readSecond = false;
    askForDist = true;

    slow_factor = 1.0;

    current_action = states[++own_state];
  }

} // end of close_in



///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Ball ... control section                      //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
void close_hatch() {        // actually Grabbing the ball. verifying with a sensor?
  /////////////////////////////////////////////////////////////////////////////////////////////////

  stopp();

  newPos = SERVO_CLOSE;
  current_action = &servo_moving;

} // end of close_hatch



///////////////////////////////////////////////////////////////////////////////////////////////////
void open_hatch() {        // actually Grabbing the ball. verifying with a sensor?
  /////////////////////////////////////////////////////////////////////////////////////////////////

  stopp();

  newPos = SERVO_OPEN;
  current_action = &servo_moving;

} // end of open_hatch



///////////////////////////////////////////////////////////////////////////////////////////////////
void servo_moving() {     // actually moving the servo
  /////////////////////////////////////////////////////////////////////////////////////////////////

  if (pos < newPos)
    pos += 10;

  if (pos > newPos)
    pos -= 10;

  myServo.write(pos);
  delay(1);

  if (pos == newPos)
    current_action = states[++own_state];

} // end of servo_moving




///////////////////////////////////////////////////////////////////////////////////////////////////
void init_prog() { // initializing (round 2)
///////////////////////////////////////////////////////////////////////////////////////////////////
  digitalWrite(LED_BUILTIN, HIGH);
  distance = sonar.ping_cm();
  digitalWrite(LED_BUILTIN, LOW);

  while (digitalRead(keysensor)) delay(20);

//   OnFwd(OUT_R, 150);
//   OnFwd(OUT_L, 150);
//
//   delay(500);
//
//   delay(500);
//
//   OFF(OUT_R);
//   OFF(OUT_L);

  delay(2000);

  askForDist = true;
  readLight = true;

  current_action = states[++own_state];
} // end of init





///////////////////////////////////////////////////////////////////////////////////////////////////
void turn() {     // Turning the robot, Software version
///////////////////////////////////////////////////////////////////////////////////////////////////
  readLight = true;
  readSecond = true;
  askForDist = false;
  analogWrite(OUT_L, (255-turnspeed));
  digitalWrite(motor_left_reverse, HIGH);
  analogWrite(OUT_R, (turnspeed));
  digitalWrite(motor_right_reverse, LOW);
  delay(50);
  while (digitalRead(lmain[4]))
    delayMicroseconds(50);
  while (!digitalRead(lmain[4]))
    delayMicroseconds(50);
  analogWrite(OUT_L, (255-turnspeed - 20));
  digitalWrite(motor_left_reverse, HIGH);
  analogWrite(OUT_R, (turnspeed - 20));
  digitalWrite(motor_right_reverse, LOW);
  while (digitalRead(lmain[0]))
    delayMicroseconds(50);
  analogWrite(OUT_L, (255-turnspeed - 40));
  digitalWrite(motor_left_reverse, HIGH);
  analogWrite(OUT_R, (turnspeed - 40));
  digitalWrite(motor_right_reverse, LOW);
  while (digitalRead(lmain[1]))
    delayMicroseconds(50);
  while (digitalRead(lmain[2]))
    delayMicroseconds(50);
  analogWrite(OUT_L, 0);
  digitalWrite(motor_left_reverse, LOW);
  analogWrite(OUT_R, 0);
  digitalWrite(motor_right_reverse, LOW);

  distance = 0;
  old_distance = 25;
  variance = 25;

  delay(50);

  readLight = true;
  readSecond = false;
  askForDist = true;
  current_action = states[++own_state];
} // end of turn




///////////////////////////////////////////////////////////////////////////////////////////////////
void finish() {     // end. done. finished task. victory.
///////////////////////////////////////////////////////////////////////////////////////////////////
  readLight = false;
  readSecond = false;
  askForDist = false;
  delay(500);
} // end of finish




///////////////////////////////////////////////////////////////////////////////////////////////////
void MotorUse(int pin, int val, bool reverse) {    // actually setting the motor
  /////////////////////////////////////////////////////////////////////////////////////////////////

  /*
  if (DEBUG) {
    Serial.print("pin: ");
    Serial.print(pin);
    Serial.print(", gesch: ");
    Serial.print(val);
    Serial.print(", rev: ");
    Serial.print(reverse);
    Serial.println("");
  }
  */

  if (reverse) {
    analogWrite(pin, 255 - val);
    digitalWrite(pin + 4, HIGH);
  } else {
    analogWrite(pin, val);
    digitalWrite(pin + 4, LOW);
  }

//  reverse ? analogWrite(pin, val), analogWrite(pin + 4, 0)
//          : analogWrite(pin, 0),   analogWrite(pin + 4, val) ;

  // analogWrite(pin, val * reverse);
  // analogWrite(pin + 4, val * (not reverse));

} // end of MotorUse



///////////////////////////////////////////////////////////////////////////////////////////////////
void error() {      // an error happened - help
///////////////////////////////////////////////////////////////////////////////////////////////////
  digitalWrite(LED_BUILTIN,HIGH);
  delay(20);
  digitalWrite(LED_BUILTIN,LOW);
  delay(20);
}



void print_mesg() {
  Serial.print("state: ");
  Serial.print(own_state);
  Serial.print(", speed: ");
  Serial.print(speed);
  Serial.print(", direction: ");
  Serial.print(direction);
  Serial.print(", old-distance: ");
  Serial.print(old_distance);
  Serial.print(", distance: ");
  Serial.print(distance);
  Serial.println("");



}

void stopp() {
  analogWrite(molipl, 0);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, 0);
  digitalWrite(moremi, LOW);
}

void geradeaus() {
  analogWrite(molipl, maxspeed * slow_factor);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, maxspeed * slow_factor);
  digitalWrite(moremi, LOW);
}

void leichtlinks() {
  analogWrite(molipl, (maxspeed-softturn * slow_factor));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, maxspeed * slow_factor);
  digitalWrite(moremi, LOW);
}

void leichtrechts() {
  analogWrite(molipl, maxspeed * slow_factor);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (maxspeed-softturn * slow_factor));
  digitalWrite(moremi, LOW);
}

void starklinks() {
  analogWrite(molipl, (255-hardturn) * slow_factor);
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (hardturn*hardturnfaktor) * slow_factor);
  digitalWrite(moremi, LOW);
}

void starkrechts() {
  analogWrite(molipl, (hardturn*hardturnfaktor) * slow_factor);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (255-hardturn) * slow_factor);
  digitalWrite(moremi, HIGH);
}

/*
void turn(){
  analogWrite(molipl, (255-turnspeed));
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (turnspeed));
  digitalWrite(moremi, LOW);
}
*/

void ruecksetzen(){
  analogWrite(molipl, (255-ruecksetzspeed) * slow_factor);
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (255-ruecksetzspeed) * slow_factor);
  digitalWrite(moremi, HIGH);
}


