
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

*/



// Final Follower - Final Code for following a line for up to 2 Arduinos,
// One of which may be controlling two motors based on a Gyro.


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

Communication:
I2C Slave, Other arduino navigating and following the line.


Written mainly by   Felix Karg     <felix.karg@uranus.uni-freiburg.de>
improvements from   Paul Boeninger <>
              and   Victor Maier   <>

For trying a Arduino-style robot at the SDP Project, University of Freiburg, WS2016/17.

License is GPL-v3.
(included in subdirectory)
*/



#include <NewPing.h>
#include <Servo.h>

#define DEBUG 1
// #define DEBUG 0

///////////////////////////////////////////////////////////////////////////////////////////////////
/*                        GENERAL INFORMATION
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


*/ // end of general information
///////////////////////////////////////////////////////////////////////////////////////////////////


// for ultrasonic sensor
#define trigPin 10
#define echoPin 11
int distance, old_distance, diff;
NewPing sonar(trigPin, echoPin, 20); // 20 cm of maximum distance we wait for the return


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

  myServo.attach(servoPin); // attaches the servo pin, for controlloing over the object now.

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









///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
//                                  Other Code                                                   //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////









//5er reihe sensoren
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
int motor_right_forward = 5;
int motor_right_reverse = 10;

// Motor Left:
int motor_left_forward = 6;
int motor_left_reverse = 11;



///////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // setting the five main light sensors as input pins
  for(int i=0; i < lmain_lenght; i++)
    pinMode(lmain[i], INPUT);

  // setting the three front light sensors as input pins
  for(int i=0; i < lfront_lenght; i++)
    pinMode(lfront[i], INPUT);

  pinMode(motor_right_forward, OUTPUT);
  pinMode(motor_right_reverse, OUTPUT);

  pinMode(motor_left_forward, OUTPUT);
  pinMode(motor_left_reverse, OUTPUT);

} // end of setup



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
void close_in() {             // closing in on the box
  /////////////////////////////////////////////////////////////////////////////////////////////////

  // TODO: get closer to the box using the ultrasonic information

} // end of close_in










///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
//                                  Other Code                                                   //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////










#include <Ultrasonic.h>

typedef struct {int direction; int speed;} motorcontrol;
typedef int (*read_pins)(uint8_t pin);
typedef void (*void_ptr)();

//array längen
#define lmain_lenght 5
#define lback_lenght 3
#define oldvar_size (1<<4)

#define STD_LOOP_TIME 49
//49= 50us loop time

#define ERROR_TIME ((STD_LOOP_TIME+1)*400)

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

#define MOTOR_STOP 0
#define MOTOR_MAX 255

// Servo winkel
#define SERVO_OPEN 138
#define SERVO_CLOSE 43
#define GREIFER_SENSOR 4
//Scan heufigkeit 1/scan_freq
#define SCANS_FREQ 10

//Pinconfig
// Motor Right:
#define motor_right_forward 5 
#define motor_right_reverse 10

// Motor Left:
#define motor_left_forward 6
#define motor_left_reverse 11

#define SCHALTER_PIN 19
#define SERVO 9
#define US_TRIGGER 12
#define US_ECHO 2

const uint8_t lback[lback_lenght]={1,/*right*/2,/*middle*/3/*left*/};
const uint8_t lmain[lmain_lenght]={7/*right*/,0/*middle*/,4/*left*/,8/*xright*/,3/*xleft*/};

const void_ptr states[]={init_prog, start_prog, line_way1, get_ball, wenden, line_way2, unget_ball, finish};
int state_index=0;

void_ptr current_state=init_prog;
int error_timer=0;
uint8_t oldvar_ptr=0;
uint8_t oldvars[oldvar_size];

Ultrasonic dist_sensor;
motorcontrol old_var;
Servo greifer;
uint8_t distance=15;

int X=0;//globale universalvariable (für nicht definierte aufgaben bei den states)




//3er reihe lichtsensoren vorne
//ANALOG
//array alte werte

// loop control
int STD_LOOP_TIME = 49; //49= 50us loop time // code that keeps loop time at 50us per cycle of main program loop
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

void init_prog(){
	distance=digitalRead_sensor(schalter);
	current_state=states[++state_index];
}

void start_prog(){
	uint8_t current_value=digitalRead(SCHALTER_PIN);
	if(current_value!=X) {current_state=states[++state_index];
		X=0;
	}else{//kann man weglassen :D
		X=current_value;
	}
}

void line_way1(){
	if(!((X++)%SCANS_US)) distance=GET_DISTANCE;//Hier fehlt ein macro
	if(digitalRead(GREIFER_SENSOR))
		set_motors(old_var=follow_line(old_var));
	current_state=states[++state_index];
}

void get_ball(){
	current_state=states[++state_index];
}

void wenden(){
	current_state=states[++state_index];
}

void line_way2(){
	if(!((X++)%SCAN_US)) distance=GET_DISTANCE;
	if(digitalRead_sensor(greifer_sensor))
		set_motors(old_var=follow_line(old_var));
	current_state=states[++state_index];
}

void unget_ball(){
	current_state=states[++state_index];
}

void finish(){
	delay(500);
}

void setup() {
  // setting the five main light sensors as input pins
  for(int i=0; i < lmain_lenght; i++) pinMode(lmain[i].pin, INPUT);
  greifer.attach(SERVO);
  // setting the three front light sensors as input pins
  for(int i=0; i < lback_lenght; i++) pinMode(lback[i].pin, INPUT);

  dist_sensor = new Ultrasonic(US_TRIGGER,US_ECHO,US_TIMEOUT);
  pinMode(motor_right_forward, OUTPUT);
  pinMode(motor_right_reverse, OUTPUT);

  pinMode(motor_left_forward, OUTPUT);
  pinMode(motor_left_reverse, OUTPUT);

} // end of setup

void loop() {         // Main Control loop
	if(error_timer)
	if((--error_timer)<1) digitalWrite(LED_BUILTIN,LOW);
  lastLoopUsefulTime = micros() - loopStartTime;

  if (lastLoopUsefulTime < STD_LOOP_TIME) { // code that keeps loop time at 50us per cycle of main program loop
    delayMicroseconds(STD_LOOP_TIME - lastLoopUsefulTime);
  }

  lastLoopTime = micros() - loopStartTime;
  loopStartTime = micros();

  current_state();


} // end of loop

void collect_ball(){ // TODO: rewrite
	greifer.write(SERVO_OPEN);
  //code hier einfügen :D
	while(digitalRead_sensor(greifer_sensor)==LOW);
	greifer.write(SERVO_CLOSE);
}

uint8_t read_X_pins (int* pins, uint8_t lenght){
  uint8_t x = 0;
  while(lenght--){
    x |= digitalRead(pins[lenght]) << lenght;
  }
  return x;
}

void error(){
	error_timer=ERROR_TIME;
	digitalWrite(LED_BUILTIN,HIGH);
}


void set_motors(motorcontrol moto) {
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

motorcontrol follow_line(motorcontrol moto) {
  uint8_t tmp=0;

	  oldvars[oldvar_ptr =
		(oldvar_ptr + 1) & (oldvar_size-1)] =
			read_X_pins(lmain, lmain_lenght) /*|(read_X_pins(lmain, lmain_lenght)<<5)*/;
	switch(tmp&0x18){ //die aeuseren
		case 0x18://00
		//moto.speed = /*const*/;
		break;

		case 0x10://01
		moto.speed = SPEED_VSLOW;
		moto.direction = DIRECTION_RIGHT_3;
		return moto;

		case 0x08://10
		moto.speed = SPEED_VSLOW;
		moto.direction = DIRECTION_LEFT_3;
		return moto;

		case 0x00://11
		error();
		moto.direction = DIRECTION_NON;
		break;
	}
	switch(tmp&0x07){
		case 0x00: //000
		//use old vars to find line
		break;

		case 0x01: //001
		moto.speed = SPEED_MIDDLE;
		moto.direction = DIRECTION_RIGHT_2;
		return moto;

		case 0x02: //010
		error();
		break;

		case 0x03: //011
		moto.speed = SPEED_FAST;
		moto.direction = DIRECTION_RIGHT_1;
		return moto;

		case 0x04: //100
		moto.speed = SPEED_MIDDLE;
		moto.direction = DIRECTION_LEFT_2;
		return moto;

		case 0x05: //101
		error();
		break;

		case 0x06: //110
		moto.speed = SPEED_FAST;
		moto.direction = DIRECTION_LEFT_1;
		return moto;

		case 0x07: //111
		//On the way
		break;
	}
} // end of follow_line

void goto_line(){
	switch((oldvars[oldvar_ptr]|=(read_X_pins(lback, lback_lenght)<<5))&0xE0){
		case 0x07: //000
		//moto.speed = /*const*/SPEED_VSLOW;
		//slow a little down
		break;

		case 0x06: //001
		//moto.speed = /*middle*/;
		//right
		break;

		case 0x05: //010
		//moto.speed = /*max*/;
		//speed up
		break;

		case 0x04: //011
		//moto.speed = /*min*/;
		//90° right
		break;

		case 0x03: //100
		//moto.speed = /*middle*/;
		//left
		break;

		case 0x02: //101
		//??
		break;

		case 0x01: //110
		//moto.speed = /*max*/;
		//90° left
		break;

		case 0x00: //111
		//??
		break;
	}
}









///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
//                                  Other Code                                                   //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


typedef struct {int direction; int speed;} motorcontrol;
typedef int (*read_pins)(uint8_t pin);
typedef void (*void_ptr)();
typedef struct {uint8_t pin; read_pins reader;} digital_read;








//c.a. 15cm
#define US_TIMEOUT 900
#define GET_DISTANCE(US_SENSOR) US_SENSOR->Ranging(1)
#define digitalRead_sensor(SENSOR) SENSOR.reader(SENSOR.pin)

#define SCHALTER_PIN 5
#define SERVO 11
#define US_TRIGGER
#define US_ECHO

const digital_read lback[lback_lenght]={{1,analogdigitalRead}/*right*/,{2,analogdigitalRead}/*middle*/,{3,analogdigitalRead/*left*/}};
const digital_read greifer_sensor = {4,digitalRead};
const digital_read lmain[lmain_lenght]={{7,digitalRead/*right*/},{0,analogdigitalRead/*middle*/},{4,digitalRead/*left*/},{8,digitalRead/*xright*/},{3,digitalRead/*xleft*/}};
const digital_read schalter={SCHALTER_PIN,analogdigitalRead}

//states array
const main_func states[]={init_prog,start_prog,line_way1,get_ball,wenden,line_way2,unget_ball,finish};

//aktuell ausgeführt
main_func current_state=init_prog;
int error_timer=0;
uint8_t oldvar_ptr=0;
uint8_t oldvars[oldvar_size];
//ANALOGPIN
Ultrasonic dist_sensor;
motorcontrol old_var;
Servo greifer;
uint8_t distance=15;

int X=0;




int analogdigitalRead(uint8_t pin){ return ((analogRead(pin)<128)?LOW:HIGH);}
//3er reihe lichtsensoren vorne
//ANALOG
//array alte werte

// loop control
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;


//States
void init_prog(){
	distance=digitalRead_sensor(schalter);
	current_state=main_func[1];
}

void start_prog(){
	uint8_t current_value=digitalRead_sensor(schalter);
	if(current_value!=X) {current_state=main_func[2];
		X=0;
	}else{//kann man weglassen :D
		X=current_value;
	}
}
void line_way1(){
	if(!((X++)%SCANS_US)) distance=GET_DISTANCE;
	if(digitalRead_sensor(greifer_sensor))
		set_motors(old_var=follow_line(old_var));
	current_state=main_func[3];
}
void get_ball(){
	current_state=main_func[3];
}
void wenden(){
	current_state=main_func[3];
}
void line_way2(){
	if(!((X++)%SCAN_US)) distance=GET_DISTANCE;
	if(digitalRead_sensor(greifer_sensor))
		set_motors(old_var=follow_line(old_var));
	current_state=main_func[3];
}
void unget_ball(){
	current_state=main_func[3];
}
void finish(){
	
}
//end States

void setup() {
  // setting the five main light sensors as input pins !NEU MACHEN WICHTIG!
  for(int i=0; i < lmain_lenght; i++) pinMode(lmain[i].pin, INPUT);
  greifer.attach(SERVO);
  // setting the three front light sensors as input pins !NEU MACHEN WICHTIG!
  for(int i=0; i < lfront_lenght; i++) pinMode(lfront[i].pin, INPUT);

  dist_sensor = new Ultrasonic(US_TRIGGER,US_ECHO,US_TIMEOUT);
  pinMode(motor_right_forward, OUTPUT);
  pinMode(motor_right_reverse, OUTPUT);

  pinMode(motor_left_forward, OUTPUT);
  pinMode(motor_left_reverse, OUTPUT);

} // end of setup

void loop() {         // Main Control loop
	if(error_timer)
	if((--error_timer)<1) digitalWrite(LED_BUILTIN,LOW);
  lastLoopUsefulTime = micros() - loopStartTime;

  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    delayMicroseconds(STD_LOOP_TIME - lastLoopUsefulTime);
  }

  lastLoopTime = micros() - loopStartTime;
  loopStartTime = micros();

  current_state();
  

} // end of loop

void collect_ball(){
	greifer.write(SERVO_OPEN);
	while(digitalRead_sensor(greifer_sensor)==LOW);
	greifer.write(SERVO_CLOSE);
}

uint8_t read_X_pins (digital_read* pins, uint8_t lenght){
  uint8_t x = 0;
  while(lenght--){
    x |= digitalRead_sensor(pins[lenght]) << lenght;
  }
  return x;
}

void error(){
	error_timer=ERROR_TIME;
	digitalWrite(LED_BUILTIN,HIGH);
}

//geht nicht rückwärz
void set_motors(motorcontrol moto) {
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

motorcontrol follow_line(motorcontrol moto) {
  uint8_t tmp=0;
  
	oldvars[oldvar_ptr =
		(oldvar_ptr + 1) & (oldvar_size-1)] = 
			read_X_pins(lmain, lmain_lenght) /*|(read_X_pins(lmain, lmain_lenght)<<5)*/;
	switch(tmp&0x18){ //die aeuseren
		case 0x18://00
		//moto.speed = /*const*/;
		break;
    
		case 0x10://01
		moto.speed = SPEED_VSLOW;
		moto.direction = DIRECTION_RIGHT_3;
		return moto;
    
		case 0x08://10
		moto.speed = SPEED_VSLOW;
		moto.direction = DIRECTION_LEFT_3;
		return moto;
    
		case 0x00://11
		error();
		moto.direction = DIRECTION_NON;
		break;
	}

	switch(tmp&0x07){
		case 0x00: //000
		//use old vars to find line
		break;
    
		case 0x01: //001
		moto.speed = SPEED_MIDDLE;
		moto.direction = DIRECTION_RIGHT_2;
		return moto;
    
		case 0x02: //010
		error();
		break;
    
		case 0x03: //011
		moto.speed = SPEED_FAST;
		moto.direction = DIRECTION_RIGHT_1;
		return moto;
    
		case 0x04: //100
		moto.speed = SPEED_MIDDLE;
		moto.direction = DIRECTION_LEFT_2;
		return moto;
    
		case 0x05: //101
		error();
		break;
    
		case 0x06: //110
		moto.speed = SPEED_FAST;
		moto.direction = DIRECTION_LEFT_1;
		return moto;
    
		case 0x07: //111
		//On the way
		break;
	}
} // end of follow_line


//komplett neu schreiben!!
void goto_line(){
	switch((oldvars[oldvar_ptr]|=(read_X_pins(lback, lback_lenght)<<5))&0xE0){
		case 0x07: //000
		//moto.speed = /*const*/SPEED_VSLOW;
		//slow a little down
		break;
		
		case 0x06: //001
		//moto.speed = /*middle*/;
		//right
		break;
		
		case 0x05: //010
		//moto.speed = /*max*/;
		//speed up
		break;
		
		case 0x04: //011
		//moto.speed = /*min*/;
		//90° right
		break;
		
		case 0x03: //100
		//moto.speed = /*middle*/;
		//left
		break;
    
		case 0x02: //101
		//??
		break;
    
		case 0x01: //110
		//moto.speed = /*max*/;
		//90° left
		break;
    
		case 0x00: //111
		//??
		break;
	}
}


