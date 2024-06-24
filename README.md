<br /> 
This repository contains code to program the MSP432P401R Microcontroller using the Arduino IDE to perform line following and maze solving.
<br /> 

There is code for both automatic solving and manual via remote control and a bluetooth connection. The line following script contains a mode selector to change the robots preferred behavior.
<br /> 

The sensors used include 2x HC-SR04 ultrasonic sensors, 6x snap action bump switches, 8x IR phototransistors, and 1x HC-05 bluetooth module. The IR sensors are used for line following, ultrasonic sensors for maze solving, and the bump switches for both. The bluetooth module can be used to get the status of the robot while it is solving as well as to use the remote control mode.
<br /> 

It is required to install the Energia MSP432 EMT RED under boards manager in the Arduino IDE. A guide to walkthrough using the board and getting it set up can be found at: https://www.energia.nu/.
<br /> 

The recommended bluetooth serial monitor is "Bluetooth Serial Terminal", which can be downloaded for free in the Microsoft store.
