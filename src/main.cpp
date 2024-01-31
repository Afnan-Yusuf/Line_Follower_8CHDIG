#include <Arduino.h>
#include <PushButton.h>
#include <TaskScheduler.h>
#include "BluetoothSerial.h"
#include "macros.h"
void btpidv(const String &data);
BluetoothSerial BT;
const byte numChars = 10;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
bool online = false;

int a, b, m, n, x, y;
char e, f, g;

float Kp = 50; 
float Ki = 0.00;
float Kd = 0;

float Kpp = 100;
float Kii = 0.00;
float Kdd = 0;
int basespeed = 170;// base speed of motor 
int minspeed = (-100);// min speed of motor
int maxspeed = 200;// max speed of motor
int T = 1;// sample time for pid calculation

#define motlf  5// motor pins
#define motlb  18
#define motrf  21
#define motrb  19
#define encr  17
#define encl  16

#define onlinesensor  35

const int freqm = 5000; // motor pwm frequency
const int mrfc = 0;// Motor Right Forward Channel (mrfc) 
const int mrbc = 2;// Motor Right Backward Channel (mrbc)
const int mlfc = 4;// Motor Left Forward Channel (mlfc)
const int mlbc = 6;//
const int mpwmr = 8;// motor pwm resolution(8bit)
int rpwm = 0; // pwm value to right motor
int lpwm = 0;//pwm value to left motor

const int irarray [8] = {13, 12, 14, 27, 26, 25, 33, 32};// ir array pins
int irval [8] = {0,0,0,0,0,0,0,0};// ir raw value
int irmin [8] = {5000,5000,5000,5000,5000,5000,5000,5000};// stores ir minim value after calib
int irmax [8] = {0,0,0,0,0,0,0,0};// stores ir maximum value after calib
int irout [8] = {0,0,0,0,0,0,0,0};// mapped ir values to usable range from min max range
const int irrange = 10; // maximum value for mapping ir value 
bool calibsts = false; // calib status
const int calres = 800;// no of samples to be taken for calibration
int center = 0;
int irdig = B00000000;
int error = 0; // position on the line from ir 

float P;// pid variables
float I;
float D;
float pidval = 0;
float lasterror;
unsigned long currenttime = 0;
int dtime = 0;
unsigned long prevt = 0;
PushButton starbutton(15);// start button pin
PushButton calbutton(2);// calibrate button pin
void readerrorb();// read error in binary
void readbutton();// read buttons
void pid_control();// pid control
void recvWithEndMarker();// bluetooth receiving
void btpidv();// assinging pid values
volatile int encoder1_value = 0;
volatile int encoder2_value = 0; // Global variable for storing the encoder position

void encoder1_isr() {
  int A = digitalRead(encl);
  if (A == HIGH) {
    encoder1_value++;
  }
}
void encoder2_isr() {
  int B = digitalRead(encr);
  if (B == HIGH) {
    encoder2_value++;
  }
}
Scheduler ts;// task scheduler object
Task readirb(TASK_IMMEDIATE, TASK_FOREVER, &readerrorb, &ts, false);// task for reading ir in binary
Task pid(TASK_IMMEDIATE, TASK_FOREVER, &pid_control, &ts, false);// task for pid calculations
Task readsw(TASK_IMMEDIATE, TASK_FOREVER, &readbutton, &ts, false);// task for reading buttons
Task btread(TASK_IMMEDIATE, TASK_FOREVER, &recvWithEndMarker, &ts, false);// task for bluetooth receiving


void readerrorb(){
  int onlineval = analogRead(onlinesensor);
  if(onlineval > 100){
    online = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    online = false;
    digitalWrite(LED_BUILTIN, LOW);
  }
  for ( byte count = 0; count < 8;count++ ){// read ir array of 8
    bitWrite(irdig, count, digitalRead( irarray[count] ));// store ir value in irdig in binary format
    //_PP(irarray[count]);
  }
  //_PL();
  switch (irdig){
    // calculating errors based on ir array outputs
    case B00011000:
    error = 0;
    break;
    case B11111111:
    error = 0;
    case B00010000:
    error = 1;
    break;
    case B00110000:
    error = 2;
    break;
    case B00100000:
    error = 3;
    break;
    case B01100000:
    error = 4;
    break;
    case B01000000:
    error = 5;
    break;
    case B11000000:
    error = 6;
    break;
    case B10000000:
    error = 7;
    break;


    case B00001000:
    error =(-1);
    break;
    case B00001100:
    error =(-2);
    break;
    case B00000100:
    error =(-3);
    break;
    case B00000110:
    error =(-4);
    break;
    case B00000010:
    error =(-5);
    break;
    case B00000011:
    error =(-6);
    break;
    case B00000001:
    error =(-7);
    break;

    case B00000111:
    error =(-7);
    break;
    case B00001111:
    error =(-7);
    break;
    case B00011111:
    error =(-7);
    break;

    case B11100000:
    error =(7);
    break;
    case B11110000:
    error =(7);
    break;
    case B11111000:
    error =(7);
    break;

    case B00000000:
    error = lasterror;
    break;

  } 
  online == true ? error = 0 : error = error;

}


void readbutton(){
starbutton.update();
  if(starbutton.isDoubleClicked()){
    if(pid.isEnabled() == false){
      pid.enable();
    }else{
      pid.disable();
      ledcWrite(mrfc, 0);// write all pwmm channel 0
      ledcWrite(mrbc, 0);
      ledcWrite(mlfc, 0);
      ledcWrite(mlbc, 0);
      P =0;
    I = 0;
    D = 0;

    }
  }
  pid.isEnabled() ? btread.disable() : btread.enable();
  ///*
  _PP(P);
  _PP("\t");
  _PP(I);
  _PP("\t");
  _PP(D);
  _PP("\t");
  _PP(error);
  _PP("\t");
  _PP(rpwm);
  _PP("\t");
  _PP(lpwm);
  _PP("\t");
  _PL("");//*/
}
String incomingData;
void recvWithEndMarker(){
  if (BT.available() > 0){
    incomingData = BT.readStringUntil('/');
    btpidv(incomingData);
  }
}

void btpidv(const String &data)
{
  _PM("Incoming Value: ");
  _PL(data);

  for (int cnt = 0; cnt <= data.length(); cnt++){
    switch (data.charAt(cnt)){
    case 'P':
      Kp = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed P value to ");
      _PL(Kp);
      BT.println(Kp);
      cnt += 5;
      break;
    case 'I':
      Ki = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed I value to ");
      _PL(Ki);
      BT.println(Ki);
      cnt += 5;
      break;
    case 'T':
      T = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed T value to ");
      _PL(T);
      BT.println(T);
      cnt += 5;
      break;
    case 'D':
      Kd = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed D value to ");
      _PL(Kd);
      BT.println(Kd);
      cnt += 5;
      break;

      case 'p':
      Kpp = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed P value to ");
      _PL(Kp);
      BT.println(Kpp);
      cnt += 5;
      break;
    case 'i':
      Kii = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed I value to ");
      BT.println(Kii);
      cnt += 5;
      break;
    case 'd':
      Kdd = data.substring(cnt + 1, cnt + 6).toFloat();
      _PM("Changed D value to ");
      _PL(Kdd);
      BT.println(Kdd);
      cnt += 5;
      break;


    case 'N':
      minspeed = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed MinSpeed value to ");
      _PL(minspeed);
      BT.println(minspeed);
      cnt += 5;
      break;
    case 'X':
      maxspeed = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed MaxSpeed value to ");
      _PL(maxspeed);
      BT.println(maxspeed);
      cnt += 5;
      break;
    case 'S':
      basespeed = data.substring(cnt + 1, cnt + 6).toInt();
      _PM("Changed Base Speed to ");
      _PL(basespeed);
      BT.println(basespeed);
      cnt += 5;
      break;
    default:
      break;
    }
  }
  P = I = D = 0;
}

void pid_control(){
   //_PP("pid: ");
    currenttime = millis();
    dtime = currenttime - prevt; 
    if (dtime >= T){
    P = error;
    I = I + error;
    D = error - lasterror;
      _PL("small error");
      pidval = ((P * Kp) + ((Ki * T) * I) + ((Kd / T) * D)); // calculater pid va
    lasterror = error;
    prevt = currenttime;
    }
  

  int lmspeed = basespeed - pidval;//get motor speed difference from pid val
  int rmspeed = basespeed + pidval;//get motor speed difference from pid val


  if (lmspeed > maxspeed){// if motor speed is greater than max speed set motor speed tot max speed
    lmspeed = maxspeed;
  }
  if (rmspeed > maxspeed){// if motor speed is greater than max speed set motor speed tot max speed
    rmspeed = maxspeed;
  }
  if(lmspeed < minspeed){// if mot speed less  than min speed set mot speed to min speed
    lmspeed = minspeed;
  }
  if(rmspeed < minspeed){// if mot speed less  than min speed set mot speed to min speed
    rmspeed = minspeed;
  }

  rpwm = abs(rmspeed);// pwm value to write to motor
  lpwm = abs(lmspeed);

  if (rmspeed > 0){ // if right mot speed is > 0 turn in forward direction else in backward 
    ledcWrite(mrfc, rpwm);
    ledcWrite(mrbc, 0);
  }else{
    ledcWrite(mrfc, 0);
    ledcWrite(mrbc, rpwm);
  }
  if (lmspeed > 0){ // if left mot speed is > 0 turn in forward direction else in backward 
    ledcWrite(mlfc, lpwm);
    ledcWrite(mlbc, 0);
  }else{
    ledcWrite(mlfc, 0);
    ledcWrite(mlbc, lpwm);
  }
}

void setup() {
  #ifdef _DEBUG_
  Serial.begin(115200);
#endif
pinMode(LED_BUILTIN, OUTPUT);
 ledcSetup(mrfc, freqm, mpwmr);// pwm channel and frequency and resolution setup
 ledcSetup(mrbc, freqm, mpwmr);
 ledcSetup(mlfc, freqm, mpwmr);
 ledcSetup(mlbc, freqm, mpwmr);
 ledcAttachPin(motrf, mrfc);// attach motor pins pwm channels
 ledcAttachPin(motrb, mrbc);
 ledcAttachPin(motlf, mlfc);
 ledcAttachPin(motlb, mlbc);
 ledcWrite(mrfc, 0);// write all pwmm channel 0
 ledcWrite(mrbc, 0);
 ledcWrite(mlfc, 0);
 ledcWrite(mlbc, 0);
 pinMode(encl, INPUT_PULLUP);
 pinMode(encr, INPUT_PULLUP);
 pinMode(onlinesensor, INPUT);
 //attachInterrupt(digitalPinToInterrupt(encl), encoder1_isr, CHANGE);
 //attachInterrupt(digitalPinToInterrupt(encr), encoder2_isr, CHANGE);
 Serial.begin(115200);
 for (int i = 0; i <= 7; i++){
    pinMode(irarray[i],INPUT);
  }
  BT.begin("LF_IR");
  starbutton.setActiveLogic(HIGH);
  calbutton.setActiveLogic(HIGH);
  ts.addTask(readirb);
  ts.addTask(pid);
  ts.addTask(readsw);
  ts.addTask(btread);
  readirb.enable();
  readsw.enable();
}

void loop() {
  ts.execute();
}

