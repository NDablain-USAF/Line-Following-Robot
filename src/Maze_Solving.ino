// Pins for left & right motors
const uint8_t Mpins[6] = {31,29,40,11,30,39}; // P3.7, 5.4, 2.7, 3.6, 5.5, 2.6
// Pins for ultrasonic sensors Echo & Trig, left & right
const uint8_t Ultra[4] = {9,35,10,15}; // P1.6, 1.7, 4.1, 1.5 
// Pins for bumpswitches
const uint8_t BS[6] = {24,25,6,27,8,28}; // P4.0, 4.2, 4.3, 4.5, 4.6, 4.7

unsigned long z = 0;
float wb = 0.08;
float umax = 0.10;
float K = 0.01;
byte ledPin = 73;
byte onetime = 0;
byte maxd = 25;
volatile byte y = 0; // Variable to hold bumpswitch actuation
byte ylast = 0; // Variable to compare bumpswitch variable to

#include "Energia.h"

long duration[2] = {0,0};
float d[2] = {0,0};

void setup() {
  Serial.begin(115200); // Begin serial communication
  Serial1.begin(9600);
  pinMode(ledPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ledPin), start, CHANGE);
  for (byte i = 0; i <= 5; i++){ // Setup motor pins
    pinMode(Mpins[i],OUTPUT); // Setup motor pins as outputs
  }
  // Setup Ultrasonic pins
  pinMode(Ultra[0],INPUT);
  pinMode(Ultra[1],INPUT);
  pinMode(Ultra[2],OUTPUT);
  pinMode(Ultra[3],OUTPUT);
  // Setup bumpswitch pins
  for (byte i = 0; i <= 5; i++){
    pinMode(BS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BS[i]),bump,CHANGE);
  }
}

void loop() {
  while(onetime == 0){ // Wait for button press to begin
    delay(1000);
  }

  bat(); // Update wall positions
  d[1] = constrain(d[1],0,maxd);
  d[0] = constrain(d[0],0,maxd);
  float x = d[1]-d[0]; // Position between walls, right is positive left is negative
  
  if (d[1]<=6)&&(d[0]<=6){ // Balance between walls
    float u = K*x;
    u = constrain(u,-umax,umax);
    float w1 = wb + u;
    float w2 = wb - u;
  }
  else if ((d[0]>=20)&&(d[1]>=20)){ // Drive straight when there are no walls nearby
    drive(wb,wb);
    delay(1200);
    z = 0;
  }
  z++;
  drive(w1,w2);

  Serial1.println(x);
}

void drive(float w1,float w2){ // Drive dammit
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

void bat(){
 digitalWrite(Ultra[2],LOW);
 delayMicroseconds(100);
 digitalWrite(Ultra[2],HIGH);
 delayMicroseconds(10);
 digitalWrite(Ultra[2],LOW);
 duration[0] = pulseIn(Ultra[0],HIGH); // Returns time of pulse in microseconds
 d[0] = (duration[0]*0.034)/2; // Distance to surface on left sensor (cm)

 digitalWrite(Ultra[3],LOW);
 delayMicroseconds(100);
 digitalWrite(Ultra[3],HIGH);
 delayMicroseconds(10);
 digitalWrite(Ultra[3],LOW);
 duration[1] = pulseIn(Ultra[1],HIGH); // Returns time of pulse in microseconds
 d[1] = (duration[1]*0.034)/2; // Distance to surface on right sensor (cm)
}

void start(){
  ++onetime;
}

void backleft(){ // Function to back up from wall and turn left
  digitalWrite(Mpins[0], HIGH); // Enable left motor
	digitalWrite(Mpins[1], HIGH); // Set left motor direction as backward
  float p1 = 0.20; // PWM percentage command
  analogWrite(Mpins[2], round(p1*255)); // Drive left motor
  digitalWrite(Mpins[3], HIGH); // Enable right motor
  digitalWrite(Mpins[4], HIGH); // Set right motor direction as backward
  analogWrite(Mpins[5], round(p1*255)); // Drive right motor
  delay(300); // Delay long enough to back up from wall
  analogWrite(Mpins[2], round(p1*255)); // Continue to drive left motor backwards
  digitalWrite(Mpins[4], LOW); // Set right motor direction as forward
  analogWrite(Mpins[5], round(p1*255)); // Drive right motor forwards to turn left
  delay(400); // Delay long enough to turn 45 degrees left
}

void bump(){ // Register a bumpswitch actuation under the variable y
  y++;
}
