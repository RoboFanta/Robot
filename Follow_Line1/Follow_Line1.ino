/*

*/

#define OUT_A 5
#define OUT_B 6
#define outpin1 5
#define outpin12 7
#define outpin2 6
#define outpin22 8


#define inpin1 9
#define inpin2 10
#define inpin3 11

int tmp = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(inpin1, INPUT);
  pinMode(inpin2, INPUT);
  pinMode(inpin3, INPUT);
  pinMode(outpin12, OUTPUT);
  pinMode(outpin22, OUTPUT);
}


void MotorUse(int pin, int speed, bool reverse){
  digitalWrite(pin + 13, reverse);
  analogWrite(pin, speed);
}

#define OnFwd(pin,speed) MotorUse(pin,speed*256.0/100.0, false)
#define OnRev(pin,speed) MotorUse(pin,speed*256.0/100.0, true)
#define Off(pin) MotorUse(pin, 0, false)


void loop() {
  // put your main code here, to run repeatedly:

  // analogRead, digitalRead, digitalWrite, analogWrite?

  tmp = 0;
  if (digitalRead(inpin1)) {
    tmp |= 1;
  }
  if (digitalRead(inpin2)) {
    tmp |= 2;
  }
  if (digitalRead(inpin3)) {
    tmp |= 4;
  }

  switch(tmp){
    case 4: //hard right
      OnFwd(OUT_B, 30);
      OnRev(OUT_A, 30);
      break;

    case 1: //hard left
      OnFwd(OUT_A, 30);
      OnRev(OUT_B, 30);
      break;

    case 6: //light left
      OnFwd(OUT_A, 20);
      OnFwd(OUT_B, 40);
      break;

    case 3: //light right
      OnFwd(OUT_A, 40);
      OnFwd(OUT_B, 20);
      break;

    case 2: /*nothing :D*/
      OnFwd(OUT_A, 45);
      OnFwd(OUT_B, 45);
      break;

//    case 0: //all 0
//    case 5: //middle nothing
//    case 7: //all 1
    default:
      delay(500);
      Off(OUT_A);
      Off(OUT_B);
      break;

  }

  delay(20);
}


