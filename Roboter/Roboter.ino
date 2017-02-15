int maxspeed = 250; //Wert von 0 bis 255
int softturn = 100; //Wert von 0 bis 255
int hardturn = 150;   //Wert von 25 bis 255
int dauerhardturn = 0; //Wert zB. 100, 0 bis 1000

int approxspd1 = 150; //Speed zwischen 15cm und 8cm Entfernung
int approxspd2 = 110; //Speed zwischen 8cm und 4cm Entfernung
int approxspd3 = 80; //Speed <4cm Entfernung
int approxsoftturn = 100; //

int ruecksetzdelay = 500; //Wie lange soll er nach dem Grabben zurückfahren (Speed = approxspd1)
int drehzyklus = 50; //Optimierung des Drehzyklus ab dem Momnt, in dem alle hinteren Sensoren "0" lesen







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
int distance = 0;                               //ULTRASCHALL

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
  
  while (startval == 1){
    startval = digitalRead(start);
    delay(50);
     servo.write(138);
  delay(300);
  servo.write(43);
  delay(500);
  }
 
  
vogalival = digitalRead(vogali);                                    //lese Sensoren aus...
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare);                                      //...und speichere sie ab

digitalWrite(ustr,LOW);
delayMicroseconds(2);
digitalWrite(ustr,HIGH);
delayMicroseconds(10);
digitalWrite(ustr,LOW);
usval = pulseIn(usec,HIGH);
distance = (usval/2)/29;

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
if (distance < 20){
  grabber();
}
delay(50);
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
  analogWrite(molipl, (280-hardturn));
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, hardturn);
  digitalWrite(moremi, LOW);
  delay(dauerhardturn);
}

void starkrechts() {
  analogWrite(molipl, hardturn);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (280-hardturn));
  digitalWrite(moremi, HIGH);
  delay(dauerhardturn);
}

void unterbrechung() {
  analogWrite(molipl, 255);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, 255);
  digitalWrite(moremi, LOW);
}

void grabber() {
  
  grsensorval = digitalRead(grsensor);
  if (grsensorval == 1){
    drop();
  }
  
  while (grsensor == 0){
    servo.write(138);
    
digitalWrite(ustr,LOW);
delayMicroseconds(2);
digitalWrite(ustr,HIGH);
delayMicroseconds(10);
digitalWrite(ustr,LOW);
usval = pulseIn(usec,HIGH);
distance = (usval/2)/29;
delay(40);

vogalival = digitalRead(vogali);
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare);  

grsensorval = digitalRead(grsensor);
  
  while (distance > 7){
if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd1);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd1);
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd1);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspd1-approxsoftturn));
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
  analogWrite(molipl, (approxspd1-approxsoftturn));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd1);
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
  starkrechts();
  analogWrite(molipl, approxspd1);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (0));
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
  analogWrite(molipl, (0));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd1);
  digitalWrite(moremi, LOW);
}
digitalWrite(ustr,LOW);
delayMicroseconds(2);
digitalWrite(ustr,HIGH);
delayMicroseconds(10);
digitalWrite(ustr,LOW);
usval = pulseIn(usec,HIGH);
distance = (usval/2)/29;

vogalival = digitalRead(vogali);
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare); 
  }
  while (distance > 3){
if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd2);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd2);
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd2);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspd2-approxsoftturn/approxspd1*approxspd2));
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
  analogWrite(molipl, (approxspd2-approxsoftturn/approxspd1*approxspd2));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd2);
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
  starkrechts();
  analogWrite(molipl, approxspd2);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (0));
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
  analogWrite(molipl, (0));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd2);
  digitalWrite(moremi, LOW);
}
digitalWrite(ustr,LOW);
delayMicroseconds(2);
digitalWrite(ustr,HIGH);
delayMicroseconds(10);
digitalWrite(ustr,LOW);
usval = pulseIn(usec,HIGH);
distance = (usval/2)/29;

vogalival = digitalRead(vogali);
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare); 
  }
  while ((distance < 4)&&(grsensor == 0)){
if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd3);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd3);
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd3);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspd3-approxsoftturn/approxspd1*approxspd3));
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
  analogWrite(molipl, (approxspd3-approxsoftturn/approxspd1*approxspd3));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd3);
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
  starkrechts();
  analogWrite(molipl, approxspd3);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (0));
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
  analogWrite(molipl, (0));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd3);
  digitalWrite(moremi, LOW);
} 
vogalival = digitalRead(vogali);
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare); 

distance = digitalRead(grsensor);
  }
  }
  
if (grsensor == 1){
  servo.write(43);
  delay(1000);
  wende();
}
}




void wende(){
  analogWrite(molipl, approxspd1);
  digitalWrite(molimi, HIGH);
  analogWrite(morepl, approxspd1);
  digitalWrite(moremi, HIGH);
  delay(ruecksetzdelay);

hilival = digitalRead(hili);
himival = digitalRead(himi);
hireval = digitalRead(hire);

while ((hili == 1)&&(himi == 0)&&(hire == 1)){
  analogWrite(molipl, (approxspd1));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (255-approxspd1));
  digitalWrite(moremi, HIGH);
  
hilival = digitalRead(hili);
himival = digitalRead(himi);
hireval = digitalRead(hire);
}
while ((hili == 0)&&(himi == 0)&&(hire == 0)){
  analogWrite(molipl, (approxspd1));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (255-approxspd1));
  digitalWrite(moremi, HIGH);
  
hilival = digitalRead(hili);
himival = digitalRead(himi);
hireval = digitalRead(hire);
delay(drehzyklus);
}
}



void drop(){
  
digitalWrite(ustr,LOW);
delayMicroseconds(2);
digitalWrite(ustr,HIGH);
delayMicroseconds(10);
digitalWrite(ustr,LOW);
usval = pulseIn(usec,HIGH);
distance = (usval/2)/29;
delay(50);

vogalival = digitalRead(vogali);
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare);  

grsensorval = digitalRead(grsensor);
  
  while (distance > 7){
if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd1);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd1);
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd1);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspd1-approxsoftturn));
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
  analogWrite(molipl, (approxspd1-approxsoftturn));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd1);
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
  starkrechts();
  analogWrite(molipl, approxspd1);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (0));
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
  analogWrite(molipl, (0));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd1);
  digitalWrite(moremi, LOW);
}
digitalWrite(ustr,LOW);
delayMicroseconds(2);
digitalWrite(ustr,HIGH);
delayMicroseconds(10);
digitalWrite(ustr,LOW);
usval = pulseIn(usec,HIGH);
distance = (usval/2)/29;
delay(40);

vogalival = digitalRead(vogali);
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare); 
  }
  while (distance > 3){
if  ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd2);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd2);
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)){
  analogWrite(molipl, approxspd2);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (approxspd2-approxsoftturn/approxspd1*approxspd2));
  digitalWrite(moremi, LOW);
}
else if ((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1)){
  analogWrite(molipl, (approxspd2-approxsoftturn/approxspd1*approxspd2));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd2);
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 1)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 1)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))
        ||((vogalival == 1)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 0))){
  starkrechts();
  analogWrite(molipl, approxspd2);
  digitalWrite(molimi, LOW);
  analogWrite(morepl, (0));
  digitalWrite(moremi, LOW);
}
else if (((vogalival == 0)&&(volival == 1)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 1)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 1)&&(vogareval == 1))
        ||((vogalival == 0)&&(volival == 0)&&(vomival == 0)&&(voreval == 0)&&(vogareval == 1)))  {
  analogWrite(molipl, (0));
  digitalWrite(molimi, LOW);
  analogWrite(morepl, approxspd2);
  digitalWrite(moremi, LOW);
}
digitalWrite(ustr,LOW);
delayMicroseconds(2);
digitalWrite(ustr,HIGH);
delayMicroseconds(10);
digitalWrite(ustr,LOW);
usval = pulseIn(usec,HIGH);
distance = (usval/2)/29;

vogalival = digitalRead(vogali);
volival = digitalRead(voli);
vomival = digitalRead(vomi);
voreval = digitalRead(vore);
vogareval = digitalRead(vogare); 
  }
  if (distance < 4){
    servo.write(138);

}
}
