#include <NewPing.h>

int maxspeed = 150; //Wert von 0 bis 255
int softturn = 40; //Wert von 0 bis 255
int hardturn = 40;   //Wert von 25 bis 255
float hardturnfaktor = 3;

int turnspeed = 150;

int approxspeed = 150;

int ruecksetzspeed = 150;
int ruecksetzdelay = 2000; //Wie lange soll er nach dem Grabben zurückfahren (Speed = approxspd1)
int drehzyklus = 200; //Optimierung des Drehzyklus ab dem Momnt, in dem alle hinteren Sensoren "0" lesen







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

int hili = 15; //ANALOG TO DIG READ
int himi = 16; //ANALOG TO DIG READ
int hire = 17; //ANALOG TO DIG READ
int hilival = 0;
int himival = 0;
int hireval = 0;                                //HINTERE SENSORREIHE

int ustr = 2; //DIG WRITE
int usec = 12; //DIG READ
long usval = 0;
int distance = 30;                              //ULTRASCHALL
int latest_dist = 30;

int grsensor = 4; //ANALOG TO DIG READ
int servopin = 11; //DIG WRITE
int grsensorval = 0;                            //GREIFER

int molipl = 6; //ANALOG WRITE
int molimi = 10; //ANALOG WRITE
int morepl = 5; //ANALOG WRITE
int moremi = 9; //ANALOG WRITE             //MOTOREN

int start = 19; //DGITAL WRITE
int startval = 1;                           //Startknopf

bool hardturnlinks = false;
bool turnlinks = false;

NewPing sonar(ustr,usec,25);

#include <Servo.h>




Servo servo;



void setup() {



  pinMode(vogali,INPUT);
  pinMode(voli,INPUT);
  pinMode(vomi, INPUT);
  pinMode(vore,INPUT);
  pinMode(vogare,INPUT);

  pinMode(hili, INPUT);
  pinMode(himi, INPUT);
  pinMode(hire, INPUT);

  pinMode(ustr,OUTPUT);
  pinMode(usec,INPUT);

  pinMode(grsensor, INPUT);
  servo.attach(servopin);

  pinMode(molipl,OUTPUT);
  pinMode(molimi,OUTPUT);
  pinMode(morepl,OUTPUT);
  pinMode(moremi,OUTPUT);

  pinMode(start, INPUT);

}

void loop() {
  servo.write(43);
  while (startval == 1){
    startval = digitalRead(start);
    delay(50);
  }


vogalival = digitalRead(vogali);                                    //lese Sensoren aus...
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare);                                      //...und speichere sie ab


distance = sonar.ping_cm();

latest_dist = distance ? distance : latest_dist;


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


if (distance < 15){
  grabber();
}
}

void stopp() {
  analogWrite(molipl, 0);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, 0);
  digitalWrite(moremi, LOW);
}

void geradeaus() {
  analogWrite(molipl, maxspeed);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, maxspeed);
  digitalWrite(moremi, LOW);
}

void leichtlinks() {
  analogWrite(molipl, (maxspeed-softturn));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, maxspeed);
  digitalWrite(moremi, LOW);
}

void leichtrechts() {
  analogWrite(molipl, maxspeed);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (maxspeed-softturn));
  digitalWrite(moremi, LOW);
}

void starklinks() {
  analogWrite(molipl, (255-hardturn));
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (hardturn*hardturnfaktor));
  digitalWrite(moremi, LOW);
}

void starkrechts() {
  analogWrite(molipl, (hardturn*hardturnfaktor));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (255-hardturn));
  digitalWrite(moremi, HIGH);
}

void turn() {
  analogWrite(molipl, (255-turnspeed));
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (turnspeed));
  digitalWrite(moremi, LOW);
  delay(50);
  while (digitalRead(vogare))
    delayMicroseconds(50);
  while (!digitalRead(vogare))
    delayMicroseconds(50);
  analogWrite(molipl, (255-turnspeed - 20));
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (turnspeed - 20));
  digitalWrite(moremi, LOW);
  while (digitalRead(voli))
    delayMicroseconds(50);
  analogWrite(molipl, (255-turnspeed - 40));
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (turnspeed - 40));
  digitalWrite(moremi, LOW);
  while (digitalRead(vomi))
    delayMicroseconds(50);
  while (digitalRead(vomi))
    delayMicroseconds(50);
  analogWrite(molipl, 0);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, 0);
  digitalWrite(moremi, LOW);
}

void ruecksetzen(){
  analogWrite(molipl, (255-ruecksetzspeed));
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, (255-ruecksetzspeed));
  digitalWrite(moremi, HIGH);
}

void grabber() {
 if (grsensorval == 1){
  drop();
 }
 else {
  while (grsensorval == 0) {
 servo.write(140);
  if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, (approxspeed));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspeed));
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1)){
  analogWrite(molipl, (0));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (0));
  digitalWrite(moremi, LOW);
 }
 else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, (approxspeed));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspeed/2));
  digitalWrite(moremi, LOW);
  turnlinks =  false;
  }
  else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
  analogWrite(molipl, (approxspeed/2));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspeed));
  digitalWrite(moremi, LOW);
  turnlinks = true;
}
else {
  if (turnlinks = true){
      analogWrite(molipl, (approxspeed));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspeed/2));
  digitalWrite(moremi, LOW);
  }
  else {
      analogWrite(molipl, (approxspeed/2));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspeed));
  digitalWrite(moremi, LOW);
  }
}
}
servo.write(43); //////////////////////////////////////////////////////////////////////////////////////////////////////
 stopp();
 }
}

void wende(){
  analogWrite(molipl, approxspeed);
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, approxspeed);
  digitalWrite(moremi, HIGH);
  delay(ruecksetzdelay);

hilival = digitalRead(hili);
himival = digitalRead(himi);
hireval = digitalRead(hire);

while ((hili == 1)&&(himi == 0)&&(hire == 1)){
  analogWrite(molipl, (approxspeed));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (255-approxspeed));
  digitalWrite(moremi, HIGH);

hilival = digitalRead(hili);
himival = digitalRead(himi);
hireval = digitalRead(hire);
}
while ((hili == 0)&&(himi == 0)&&(hire == 0)){
  analogWrite(molipl, (approxspeed));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (255-approxspeed));
  digitalWrite(moremi, HIGH);

hilival = digitalRead(hili);
himival = digitalRead(himi);
hireval = digitalRead(hire);
delay(drehzyklus);
}
}



void drop(){

//digitalWrite(ustr,LOW);
//delayMicroseconds(2);
//digitalWrite(ustr,HIGH);
//delayMicroseconds(10);
//digitalWrite(ustr,LOW);
//usval = pulseIn(usec,HIGH);
//distance = (usval/2)/29;
//delay(50);
//
//vogalival = digitalRead(vogali);
//volival = digitalRead(voli);
//vomival = digitalRead(vomi);
//voreval = digitalRead(vore);
//vogareval = digitalRead(vogare);
//
//grsensorval = digitalRead(grsensor);
//
//  while (distance > 7){
//if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
//  analogWrite(molipl, approxspd1);
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, approxspd1);
//  digitalWrite(moremi, LOW);
//}
//else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
//  analogWrite(molipl, approxspd1);
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, (approxspd1-approxsoftturn));
//  digitalWrite(moremi, LOW);
//}
//else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
//  analogWrite(molipl, (approxspd1-approxsoftturn));
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, approxspd1);
//  digitalWrite(moremi, LOW);
//}
//else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
//        ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
//        ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
//        ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
//  analogWrite(molipl, approxspd1);
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, (0));
//  digitalWrite(moremi, LOW);
//}
//else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
//        ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
//        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
//        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
//  analogWrite(molipl, (0));
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, approxspd1);
//  digitalWrite(moremi, LOW);
//}
//digitalWrite(ustr,LOW);
//delayMicroseconds(2);
//digitalWrite(ustr,HIGH);
//delayMicroseconds(10);
//digitalWrite(ustr,LOW);
//usval = pulseIn(usec,HIGH);
//distance = (usval/2)/29;
//delay(40);
//
//vogalival = digitalRead(vogali);
//volival = digitalRead(voli);
//vomival = digitalRead(vomi);
//voreval = digitalRead(vore);
//vogareval = digitalRead(vogare);
//  }
//  while (distance > 3){
//if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
//  analogWrite(molipl, approxspd2);
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, approxspd2);
//  digitalWrite(moremi, LOW);
//}
//else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
//  analogWrite(molipl, approxspd2);
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, (approxspd2-approxsoftturn/approxspd1*approxspd2));
//  digitalWrite(moremi, LOW);
//}
//else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
//  analogWrite(molipl, (approxspd2-approxsoftturn/approxspd1*approxspd2));
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, approxspd2);
//  digitalWrite(moremi, LOW);
//}
//else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
//        ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
//        ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
//        ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
//  analogWrite(molipl, approxspd2);
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, (0));
//  digitalWrite(moremi, LOW);
//}
//else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
//        ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
//        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
//        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
//  analogWrite(molipl, (0));
//  digitalWrite(molimi, LOW);
//  analogWrite(morepl, approxspd2);
//  digitalWrite(moremi, LOW);
//}
//digitalWrite(ustr,LOW);
//delayMicroseconds(2);
//digitalWrite(ustr,HIGH);
//delayMicroseconds(10);
//digitalWrite(ustr,LOW);
//usval = pulseIn(usec,HIGH);
//distance = (usval/2)/29;
//
//vogalival = digitalRead(vogali);
//volival = digitalRead(voli);
//vomival = digitalRead(vomi);
//voreval = digitalRead(vore);
//vogareval = digitalRead(vogare);
//  }
//  if (distance < 4){
//    servo.write(138);
//    delay(500);
//wende();
//}
}
