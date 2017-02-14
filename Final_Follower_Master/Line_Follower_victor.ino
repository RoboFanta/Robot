#include <Servo.h>
#include <Ultrasonic.h>

typedef struct {int direction; int speed;} motorcontrol;
typedef int (*read_pins)(uint8_t pin);
typedef void (*main_func)();
typedef struct {uint8_t pin; read_pins reader;} digital_read;

//array längen
#define lmain_lenght 5
#define lback_lenght 3
#define oldvar_size (1<<4)

#define STD_LOOP_TIME 49
//49= 50us loop time // code that keeps loop time at 50us per cycle of main program loop


#define ERROR_TIME ((STD_LOOP_TIME+1)*400)

#define SPEED_VSLOW 48
#define SPEED_SLOW 92
#define SPEED_MIDDLE 128
#define SPEED_FAST 178
#define SPEED_VFAST 255

#define DIRECTION_RIGHT_1 31
#define DIRECTION_RIGHT_2 63
#define DIRECTION_RIGHT_3 127
#define DIRECTION_LEFT_1 -31
#define DIRECTION_LEFT_2 -63
#define DIRECTION_LEFT_3 -127

#define MOTOR_STOP 0
#define MOTOR_MAX 255
#define DIRECTION_NON 0

//servo winkel
#define SERVO_OPEN 138
#define SERVO_CLOSE 43
//Scan heufigkeit 1/scan_us
#define SCANS_US 10

//c.a. 15cm
#define US_TIMEOUT 900
#define GET_DISTANCE(US_SENSOR) US_SENSOR->Ranging(1)
#define digitalRead_sensor(SENSOR) SENSOR.reader(SENSOR.pin)
//Pinconfig
// Motor Right:
#define MRFORWARD 5
#define MRREVERSE 10
// Motor Left:
#define MLFORWARD 5
#define MLREVERSE 11

#define SCHALTER_PIN 5
#define SERVO 12
#define US_TRIGGER 2
#define US_ECHO 11

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
  pinMode(MRFORWARD, OUTPUT);
  pinMode(MRREVERSE, OUTPUT);

  pinMode(MLFORWARD, OUTPUT);
  pinMode(MLREVERSE, OUTPUT);

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
			analogWrite(MLFORWARD,((int)(moto.speed)+(int)(moto.direction)<MOTOR_STOP)? MOTOR_STOP : (moto.speed+moto.direction) );
			analogWrite(MRFORWARD,((int)(moto.speed)-(int)(moto.direction)>MOTOR_MAX)? MOTOR_MAX : (moto.speed-moto.direction) );
			analogWrite(MLREVERSE,((int)(moto.speed)+(int)(moto.direction)< MOTOR_STOP)?  -(moto.speed+moto.direction) : MOTOR_STOP) ;
			analogWrite(MRREVERSE,MOTOR_STOP);
		}
		else{
			analogWrite(MLFORWARD,((int)(moto.speed)+(int)(moto.direction)>MOTOR_MAX)?MOTOR_MAX:moto.speed+moto.direction);
			analogWrite(MRFORWARD,((int)(moto.speed)-(int)(moto.direction)<MOTOR_STOP)?MOTOR_STOP:moto.speed-moto.direction);
			analogWrite(MLREVERSE,MOTOR_STOP);
			analogWrite(MRREVERSE,((int)(moto.speed)-(int)(moto.direction) < MOTOR_STOP) ? -(moto.speed-moto.direction) : MOTOR_STOP);
		}
  }
  else{
  		analogWrite(MRFORWARD,moto.speed);
		analogWrite(MLFORWARD,moto.speed);
		analogWrite(MRREVERSE,MOTOR_STOP);
  		analogWrite(MLREVERSE,MOTOR_STOP);
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
