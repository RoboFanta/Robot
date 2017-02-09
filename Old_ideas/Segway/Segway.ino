boolean debugMode = false; //set to true to execute Serial debug data

#include "Wire.h"

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars

bool dmpReady = false; // set true if DMP init was successful

uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU

uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)

uint16_t packetSize; // expected DMP packet size (default is 42 bytes)

uint16_t fifoCount; // count of all bytes currently in FIFO

uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars

Quaternion q; // [w, x, y, z] quaternion container

VectorInt16 aa; // [x, y, z] accel sensor measurements

VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements

VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements

VectorFloat gravity; // [x, y, z] gravity vector

float euler[3]; // [psi, theta, phi] Euler angle container

float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo

uint8_t teapotPacket[14] = {

'$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================

// === INTERRUPT DETECTION ROUTINE ===

// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {

mpuInterrupt = true;

}

// ================================================================

// === SEGWAY SETUP ===

// ================================================================

#include <SoftwareSerial.h>

SoftwareSerial Segway(12,4); //(4, 12); // RX, TX, (D12 is used to send commands to the Sabertooth)

boolean statusOK = false; //will be set to true once the segway is level for startup

float pitch;

int prevPitch;

int motor_speed; //speed value used to give values to each motor

int calibrateButton = A1; //pushbutton connected to A1 pin

int pitchOffset = 0; //in case the IMU is not physically mounted perpedicular

int maxSpeed = 64; //will be changed by standbyMode

boolean standbyMode = true;

// ================================================================

// === INITIAL SETUP ===

// ================================================================

void setup() {

// join I2C bus (I2Cdev library doesn't do this automatically)

Wire.begin();

Serial.begin(115200); //for debugging

Segway.begin(9600); //begin communication with Segway motor driver

// initialize device

Serial.println(F("Initializing I2C devices..."));

mpu.initialize();

// verify connection

Serial.println(F("Testing device connections..."));

Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// load and configure the DMP

Serial.println(F("Initializing DMP..."));

devStatus = mpu.dmpInitialize();

// make sure it worked (returns 0 if so)

if (devStatus == 0) {

// turn on the DMP, now that it's ready

Serial.println(F("Enabling DMP..."));

mpu.setDMPEnabled(true);

// enable Arduino interrupt detection

Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

attachInterrupt(0, dmpDataReady, RISING);

mpuIntStatus = mpu.getIntStatus();

// set our DMP Ready flag so the main loop() function knows it's okay to use it

Serial.println(F("DMP ready! Waiting for first interrupt..."));

dmpReady = true;

// get expected DMP packet size for later comparison

packetSize = mpu.dmpGetFIFOPacketSize();

}

else {

// ERROR!

// 1 = initial memory load failed

// 2 = DMP configuration updates failed

// (if it's going to break, usually the code will be 1)

Serial.print(F("DMP Initialization failed (code "));

Serial.print(devStatus);

Serial.println(F(")"));

}

Serial.println(F("Waiting for Segway to be balanced before starting"));

}

// ================================================================

// === MAIN PROGRAM LOOP ===

// ================================================================

void loop() {

// if programming failed, don't try to do anything

if (!dmpReady) return;

// wait for MPU interrupt or extra packet(s) available

while (!mpuInterrupt && fifoCount < packetSize) {

// other program behavior stuff here doesn't work!

}

// reset interrupt flag and get INT_STATUS byte

mpuInterrupt = false;

mpuIntStatus = mpu.getIntStatus();

// get current FIFO count

fifoCount = mpu.getFIFOCount();

// check for overflow (this should never happen unless our code is too inefficient)

if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

// reset so we can continue cleanly

mpu.resetFIFO();

if (debugMode) Serial.println(F("FIFO overflow!"));

// otherwise, check for DMP data ready interrupt (this should happen frequently)

}

else if (mpuIntStatus & 0x02) {

// wait for correct available data length, should be a VERY short wait

while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

// read a packet from FIFO

mpu.getFIFOBytes(fifoBuffer, packetSize);

// track FIFO count here in case there is > 1 packet available

// (this lets us immediately read more without waiting for an interrupt)

fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL

mpu.dmpGetQuaternion(&q, fifoBuffer);

mpu.dmpGetGravity(&gravity, &q);

mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

pitch = (float(ypr[1] * 180/M_PI)); //angle of platform

pitch = pitch - pitchOffset; //compensate for platform level point

pitch = pitch * 100;

pitch = int(pitch);

#endif

}

if (statusOK) update_motor();

else startup(); //make sure it's level before the intitial start

// leds();

}

//try changing the loop time, the maxPitch to 1300; the fscale factor to 2.5 or 3

void update_motor(){ // Update the motors

static unsigned long prevTime;

int motor1, motor2;

if (millis() - prevTime < 30) return; //only do this every 30 ms

prevTime = millis(); //update the time stamp

//modeCheck(); //update maxPitch for standby mode

int maxPitch = 1500; //value from the MPU-6050, this makes 30 degrees the max tilt for 100% speed

int maxSpeed = 64;

if (pitch < -3000) crash(); //we probably crashed

if (pitch > 3000) crash(); //we probably crashed

//motor_speed = map(pitch, -maxPitch, maxPitch, -standbySpeed, standbySpeed); //standby mode

motor_speed = map(pitch, -maxPitch, maxPitch, -maxSpeed, maxSpeed); //it's now 1 to 64

if (motor_speed > 0) motor_speed = fscale( 1, maxSpeed, 1, maxSpeed, motor_speed, -2); //scale it to change slightly at low numbers

if (motor_speed < 0) motor_speed = fscale( -maxSpeed,-1, -maxSpeed,-1,motor_speed, 2); //scale it to change slightly at low numbers

//if (motor_speed > 45) motor_speed = 64; //that's the maximum speed

Serial.println(motor_speed);

// if (CheckPosNeg(motor_speed)) { //if true, we're oscilating out of control

// int a = 0;

// Segway.write(a);

// statusOK = false;

// return;

// }

motor1 = motor_speed + get_Steering(); //add steering bias to motor 1

motor2 = motor_speed - get_Steering(); //add steering bias to motor 2

// assign final motor output values

motor1 = 65 + motor1; //64 is neutral for motor 1

motor2 = 192 + motor2; //192 is neutral for motor 2

motor1 = constrain(motor1, 1, 127); //constrain the value to it's min/max

motor2 = constrain(motor2, 128, 255);//constrain the value to it's min/max

if (!debugMode) Segway.write(motor1); //Send motor 1 speed over serial

delay(1); //sabertooth can only receive commands at 2000/second

if (!debugMode) Segway.write(motor2); //Send motor 2 speed over serial

if (debugMode) {

Serial.print("Pitch: ");

Serial.print(pitch);

Serial.print("\tMotorSpeed1: ");

Serial.print(motor1);

Serial.print("\tMotorSpeed2: ");

Serial.println(motor2);

}

}

void crash() {

Serial.print("CRASHED!");

int crashed = 0;

while(1){

Segway.write(crashed); //Send motor stop command to inhibit further wounds

delay(50);

}

}

void startup(){

//give the IMU a chance to be up and running

if (millis() < 1000) return;

int tilt = abs(pitch);

if (tilt < 10) { //don't start until it is balanced!

statusOK = true;

Serial.println("Segway is balanced");

}

}

int get_Steering() {

//0 to 1023 is the full reading of POT

int reading = analogRead(A0);

int speedVal = abs(motor_speed);

if (speedVal > 15) reading = map(reading, 0, 1023,-6 , 6); //smoothe turning at higher speeds

else reading = map(reading, 0, 1023,-10 , 10); //this gives a nicer turn at low speeds

return reading;

}

unsigned long prevChangeTime; //last time we changed directions

unsigned long lastRevTime; //last time we went into reverse

const boolean forwardDirection = true;

const boolean reverseDirection = false;

boolean CheckPosNeg(int Val) {

//return false;

const int ControlState = -2;

if (Val == ControlState) return false; //when it's 0, it can be ignored

int timeRange = 2000; //within this time range

const int changeCount = 3; //number of changes to trigger flag

static int lastVal; //the last value passed

static int countArray(changeCount); //I need to keep track of this many changes

static unsigned long timeArray[changeCount + 1];//this will hold a time stamp for each change {1,2,3}

//Serial.println(Val);

if (Val > ControlState) {

if(lastVal < ControlState) { //a change

for (int i=changeCount; i > 0; i--){

timeArray = timeArray[i-1]; //move all the value up the line

}

timeArray[0] = millis(); //put the current one at the beginning

lastVal = Val; //update the lastVal

//now see if it's been longer than [timeRange] for the least recent timestamp

//Serial.println(timeArray[changeCount]);

if (millis() - timeArray[changeCount] < timeRange) {

Serial.println("Oscilate!");

return true; //send up red sparks!

}

else return false; //all is well

}

}

lastVal = Val;

return false;

}
