// Pins for left & right motors
const uint8_t Mpins[6] = {31,29,40,11,30,39}; // P3.7, 5.4, 2.7, 3.6, 5.5, 2.6, Enable/disable, direction, duty cycle
// Pins for IR emitters
const uint8_t IR[6] = {41,42,43,44,45,46};// P7.0, 7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.7, Right to left side of robot

#include "Energia.h" // Include Energia library so that this code can be run in Arduino IDE
 
char Command; // Command received via bluetooth for remote control, w to go straight at intersections, d to turn right at intersections, a to turn left at intersections,
// s to stop immediately and turnaround

float IRp[6] = {0,0,0,0,0,0}; // Initializing the post calibration sensor readings
float IRw[6] = {1.0,0.66,0.33,-0.33,-0.66,-1.0}; // Weights found from measurements of sensor bar 
float c[2] = {0.5,0.5}; // Constant to filter calibration data, current reading weight and past reading weight
float Kp = 0.75; // Proportional control gain
float wb = 0.15; // Baseline motor speed
float wrange = 0.75; // Limit of control input allowed 
float bias = 4; // Bias in state measurement
int IRrange[6] = {1,1,1,1,1,1}; // Initializing range of IR sensor readings once offset has been removed
int IRvals[6] = {0,0,0,0,0,0};  // Initializing array for reading IR sensor values
int IRoffset[6] = {0,0,0,0,0,0}; // Initializing array for holding the offset found during calibration
int fudge = 900; // Fudge factor to drive sensors below 0 when reading nothing
int tcal = 3000; // Time for calibration, in ms
byte control[3] = {0,0,0}; // Variables to control state of system

void setup() {
  Serial1.begin(9600);
  for (byte i = 0; i <= 5; i++){ // Setup motor pins
    pinMode(Mpins[i],OUTPUT); // Setup motor pins as outputs
  }
  // IR pinmode set in loop
}

void loop() {
  sens(); // Update IR sensor reading

  while ((Command!='r')&&(control[0]==0)){
    stop();
    if (control[1]==0){
      Serial1.println("Press r to begin line following");
      delay(1000);
      Serial1.println("Press w to go straight at intersections press d to go right at intersections");
      Serial1.println("Press a to go left at intersections, and press s to turnaround");
      delay(1000);
      Serial1.println("Press t to stop the robot");
      delay(1000);
      control[1] = 1;
    }
    while (Serial1.available()>0){
      Command = Serial1.read();
    }
    if (Command=='r'){
      control[0]++;
    }
  }

  long ts = millis(); // Time at start of calibration
  while(control[2] == 0){ // Calibrate IR sensors 
    sens(); // Activate IR array
    forward(); // Drive slowly forward 
    for (int i=0;i<sizeof(IR);i++){
      IRoffset[i] = IRvals[i]*c[0] + IRoffset[i]*c[1]; // Calculate offset from 0 with weight factors c1 & c2 applied against previous and current values to filter data
    }
    if (millis()-ts > tcal){ // Once the calibration time has passed
      for (int i=0;i<sizeof(IR);i++){
        IRoffset[i] = fudge + IRoffset[i]; // Use current IRoffset, drive readings of no line below 0 with fudge factor
        IRrange[i] = 3500-IRoffset[i]; // Find full range of IR values for individual sensor, used to normalize later.
      }
      stop(); // Stop moving foward
      control[2] = 1; // Variable to break while loop
    }
  }

  while (Serial1.available()>0){
    Command = Serial1.read();
  }

  float Itot = 0; // Total sensor bar readings, initialize upcoming for loop
  float dot = 0; // Variable to hold dot product, initialize upcoming for loop
  for (int i=0;i<sizeof(IR);i++){ // Find sum of weights along sensor bar, implement command
    IRp[i] = IRvals[i]-IRoffset[i]; // Pull out sensor bias
    if (IRp[i] < 0){ // Saturate at 0
      IRp[i] = 0;
    }
    else{
      IRp[i] = (IRp[i]/IRrange[i]); // Normalize output to range of 0-1 with floating point precision
    }
    if (Command == 'w'){ // If command is to go straight
      IRp[2] = (bias-1)*IRp[2]; // Bias center-left
      IRp[3] = bias*IRp[3]; // Bias center-right
    }
    else if (Command == 'd'){ // If command is to turn right at intersections 
      IRp[0] = bias*IRp[0]; // Bias to turn right
      IRp[1] = (bias-1)*IRp[1]; // Bias to turn right
    }
    else if (Command == 'a'){ // If command is to turn left at intersections
      IRp[5] = (bias-1)*IRp[5]; // Bias to turn left
      IRp[6] = bias*IRp[6]; // Bias to turn left
    }
    Itot = Itot + IRp[i]; // Sum of sensor measurements
    dot = IRp[i]*IRw[i] + dot; // Dot product of sensor weights with sensor measurements
  }
  float x = (dot/Itot); // // Position of line under bar, 1DOF system with origin at the center of the bar and positive x pointing to the right from a top down perspective
  if (Itot == 0){ // In case of division by 0
    x = 0;
  }
  float u1 = Kp*x; // Control input to left motor
  float u2 = -Kp*x; // Control input to right motor
  constrain(u1, -wrange, wrange); // Saturate control input
  constrain(u2, -wrange, wrange); // Saturate control input
  float w2 = u2 + wb; // Right motor duty cycle
  float w1 = u1 + wb; // Left motor duty cycle

  if (Command=='s'){ // If commanded to turnaround, turnaround and then reset to driving straight
    turnaround();
    Command = 'w';
  }
  else if (Command=='t'){
    control[0] = 0;
    control[1] = 0;
  }

  drive(w1,w2); // Send signals to motors
  Serial1.println(x);
}

void drive(float w1,float w2){ // Drive function for sending control signals to wheels
  if (w1<0){ // Decide motor direction
    digitalWrite(Mpins[1],HIGH); // Set left motor backward if desired speed is negative
  }
  if (w1>=0){
    digitalWrite(Mpins[1],LOW); // Set left motor forward if desired speed is positive
  }
  digitalWrite(Mpins[0], HIGH); // Enable left motor
  analogWrite(Mpins[2], round(abs(w1)*255)); // Implement duty cycle on left motor
  digitalWrite(Mpins[3], HIGH); // Enable right motor
	if (w2<0){
    digitalWrite(Mpins[4],HIGH); // Set right motor direction backward if desired speed is negative
  }
  if (w2>=0){
    digitalWrite(Mpins[4],LOW); // Set right motor direction forward if desired speed is positive
  }
  analogWrite(Mpins[5], round(abs(w2)*255)); // Implement duty cycle on right motor
}

void sens(){ // Pull in sensor data from IR sensors
  for (int i=0;i<sizeof(IR);i++){
    IRvals[i] = 3500; // Initialize readings, this will be the return if no reflection is received
    pinMode(IR[i], OUTPUT); // Prepare to emit
    digitalWrite(IR[i], HIGH); // Emit
  }
  delayMicroseconds(10); // Time taken for sensor to fully emit
  noInterrupts(); // Disable interrupts to turn whole bar off at once
  long t1 = micros(); // Initialize current time
  long t2 = 0; // Initialize future time 
  for (int i=0;i<sizeof(IR);i++){
    pinMode(IR[i], INPUT); // Change emitters to receivers
  }
  while (t2 < 3500){
    t2 = micros() - t1;
    for (int i=0;i<sizeof(IR);i++){
      if ((digitalRead(IR[i]) == LOW) && (t2 < IRvals[i])){
        IRvals[i] = t2;
      }
    }
    interrupts(); // re-enable
  }
}

void forward(){ // Go forward
  digitalWrite(Mpins[0], HIGH); // Enable left motor
	digitalWrite(Mpins[1], LOW); // Set left motor direction as clockwise
  float p1 = 0.07;
  analogWrite(Mpins[2], round(p1*255));
  digitalWrite(Mpins[3], HIGH); // Enable right motor
	digitalWrite(Mpins[4], LOW); // Set right motor direction as clockwise
  analogWrite(Mpins[5], round(p1*255));
}

void stop(){ // Disable the motors to stop the robot 
  digitalWrite(Mpins[0], LOW); // Disable left motor
  digitalWrite(Mpins[3], LOW); // Disable right motor
}

void turnaround(){ // Backup and turn around 180 degrees
  stop(); // Disable motors
  digitalWrite(Mpins[0], HIGH); // Enable left motor
	digitalWrite(Mpins[1], HIGH); // Set left motor direction as counterclockwise
  float p1 = 0.10; // PWM command percentage
  analogWrite(Mpins[2], round(p1*255));
  digitalWrite(Mpins[3], HIGH);
  digitalWrite(Mpins[4], HIGH);
  analogWrite(Mpins[5], round(p1*255));
  delay(1000); // Delay to allow enough time for the robot to leave the wall in reverse
  stop(); // Disable motors
  digitalWrite(Mpins[0], HIGH); // Enable left motor
	digitalWrite(Mpins[1], LOW); // Set left motor direction as clockwise
  p1 = 0.20; // New PWM command percentage
  analogWrite(Mpins[2], round(p1*255));
  digitalWrite(Mpins[3], HIGH);
  digitalWrite(Mpins[4], HIGH);
  analogWrite(Mpins[5], round(p1*255));
  delay(1200); // Delay to allow the robot enough time to turn around
  stop(); // Disable motors
}