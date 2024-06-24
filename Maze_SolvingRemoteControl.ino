// Pins for left & right motors
const uint8_t Mpins[6] = {31,29,40,11,30,39}; // P3.7, 5.4, 2.7, 3.6, 5.5, 2.6
// Pins for ultrasonic sensors Echo & Trig, left & right
const uint8_t Ultra[4] = {9,35,10,15}; // P1.6, 1.7, 4.1, 1.5 

char Command; // Command received via bluetooth for remote control, w to balance between walls, d to turn right, a to turn left, s to turn around.

long duration[2] = {0,0}; // Array to hold wait time of ultrasonic sensor pulse
float d[2] = {0,0}; // Array to hold distance calculated from ultrasonic sensor pulse length
float w1 = 0; // Initialize left wheel speed
float w2 = 0; // Initialize right wheel speed
byte control[2] = {0,0}; // Variable to hold start and stop signal

#include "Energia.h"

void setup() {
  Serial1.begin(9600);
  // Setup Motor pins
  for (byte i = 0; i <= 5; i++){ 
    pinMode(Mpins[i],OUTPUT); // Setup motor pins as outputs
  }
  // Setup Ultrasonic pins
  pinMode(Ultra[0],INPUT);
  pinMode(Ultra[1],INPUT);
  pinMode(Ultra[2],OUTPUT);
  pinMode(Ultra[3],OUTPUT);
}

void loop() {

  while (Serial1.available()>0){
    Command = Serial1.read();
  }

  while ((Command!='r')&&(control[0]==0)){
      stop();
      if (control[1]==0){
        Serial1.println("Press r to begin maze solving");
        delay(1000);
        Serial1.println("Press d to go right, a to go left, and s to go backward");
        delay(1000);
        Serial1.println("Press t to stop the robot");
        delay(1000);
        control[1] = 1;
      }
      while (Serial1.available()>0){
        Command = Serial1.read();
      }
      if (Command=='r'){
        control[0] = 1;
      }
  }

  if (Command == 'w'){
    bat();
    float x = constrain(d[1]-d[0],-1,1);
    float K = 0.01;
    w1 = 0.10 + K*x;
    w2 = 0.10 - K*x;
  }
  else if (Command == 'd'){
    w1 = 0.20;
    w2 = -0.20;
    drive(w1,w2);
    delay(450);
  }
  else if (Command == 'a'){
    w1 = -0.20;
    w2 = 0.20;
    drive(w1,w2);
    delay(450);
  }
  else if (Command == 's'){
    w1 = 0.40;
    w2 = -0.40;
    drive(w1,w2);
    delay(750);
  }
  else if (Command == 't'){
    control[0] = 0;
    control[1] = 0;
  }
  Command = 'w';

  drive(w1,w2);
}

void drive(float w1,float w2){ // Input desired speeds to motors
  digitalWrite(Mpins[0], HIGH); // Enable left motor
  digitalWrite(Mpins[1],LOW); // Set left motor direction as forward
  if (w1<0){
    digitalWrite(Mpins[1], HIGH); // Set left motor direction as backward
  }
  analogWrite(Mpins[2], round(abs(w1)*255));
  digitalWrite(Mpins[3], HIGH); // Enable right motor
  digitalWrite(Mpins[4],LOW); // Set right motor direction as forward
  if (w2<0){
    digitalWrite(Mpins[4],HIGH); // Set right motor direction as backward
  }
  analogWrite(Mpins[5], round(abs(w2)*255));
}

void bat(){ // Get ultrasonic sensor readings 
 digitalWrite(Ultra[2],LOW); // Prepare to send wave
 delayMicroseconds(100);
 digitalWrite(Ultra[2],HIGH); // Send wave, long enough to fully transmit
 delayMicroseconds(10);
 digitalWrite(Ultra[2],LOW); // Stop sending wave and prepare to listen
 duration[0] = pulseIn(Ultra[0],HIGH); // Returns time of pulse in microseconds
 d[0] = (duration[0]*0.034)/2; // Distance to surface on left sensor (cm)

 digitalWrite(Ultra[3],LOW); // Prepare to send wave
 delayMicroseconds(100);
 digitalWrite(Ultra[3],HIGH); // Send wave, long enough to fully transmit
 delayMicroseconds(10);
 digitalWrite(Ultra[3],LOW); // Stop sending wave and prepare to listen
 duration[1] = pulseIn(Ultra[1],HIGH); // Returns time of pulse in microseconds
 d[1] = (duration[1]*0.034)/2; // Distance to surface on right sensor (cm)
}

void stop(){
  digitalWrite(Mpins[0],LOW);
  digitalWrite(Mpins[3],LOW);
}