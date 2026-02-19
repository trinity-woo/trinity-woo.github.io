+++
title = "Lab 3: Time of Flight Sensors"
date = 2026-02-10
template = "page.html"
+++

Adding the Time-of-Flight (ToF) sensors, modifying battery wiring, and testing new ToF and IMU data. <!-- more -->

### Prelab

The ToF sensor were familiarized with by reading its manual and datasheet. In the technical specifications, the $I^2C$ is described to have a "Up to 400 kHz serial bus programmable address. Default is **0x52**."

Since the two ToF sensors have the same hardwired address, we must change one to read all the data. We can either change one of the addresses programmatically or control both through the shutdown (XSHUT) pins. I decided to go with the programmatic method, since we can change the address of the sensor once at power up (by connecting the XSHUT pin to a separate GPIO pin) instead of continuously enabling/disabling the pins.

Next, the placement of the two sensors must be planned. One sensor is placed in the front of the car to detect obstacles as soon as possible as it will mostly drive forward. The second sensor was placed on the side (right) so that a wider field of view is covered, and a sensor on the back would be unncessary.

A wiriing diagram is shown below:

! diagram

Briefly discuss the approach to using 2 ToF sensors
Briefly discuss placement of sensors on robot and scenarios where you will miss obstacles
Sketch of wiring diagram (with brief explanation if you want)

### Connecting Sensors and Soldering 

First, the 650mAh battery was wired to the JST jumper wires and the Artemis. The polarity of the JST connector was black to positive and red to negative, so opposite color wires were soldered. The Artemis is now powered using only the battery. 

Next, the ToF sensors are soldered to the long QWIIC cables. The VIN, GND, SDA, and SCL pins are soldered to red, black, blue, and yellow respectively.

The Artemis, IMU, and ToF sensors were all connected to the QWIIC break-out board as shown below:

!image

### Testing the Sensor Modes

To find the sensor, the *Example05_wire_I2C* code was run. The address was printed:

The sensor modes (short, medium, and long) are optimized based on the maximum expected ranges of 1.3m, 3m, and 4m. Each mode is useful in different scenarios depending on how far we expect our obstacles to be. For example, in a closed-space or a maze, the short mode is useful. Since our robot will be designed to scan the room and maintain position with respect to walls, the long mode is chosen.

### 2 ToF Sensors and IMU


### Collaborations

(https://boltstrike.github.io/pages/lab3.html) ToF wiring soldering setup

Katherine for prelab start up address 
