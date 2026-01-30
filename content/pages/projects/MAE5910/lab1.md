+++
title = "Lab 1: Artemis and Bluetooth"
date = 2026-01-23
template = "page.html"
+++

Setting up the Artemis board and sending bluetooth data between the computer and the robot. <!-- more -->

## Lab 1A
### Prelab
Setup included updating the ArduinoIDE and installing the custom Sparkfun Apollo3 board definition for development with the Artemis. 

### 1 Artemis Connection

The Artemis board was connected to the computer via USB-C, and the *Redboard Artemis Nano* board and corresponding port were selected.

![](/lab1/artemis.png)

### 2 Blink

The *Blink* example code was compiled and uploaded. The resulting blinking LED on the Artemis is shown below. 

<video src="/lab1/blink.mp4" muted loop autoplay></video>
### 3 Serial Monitor

The *Example4_Serial* code was compiled and uploaded. The serial monitor's baud rate was set to 115000 to match the rate defined in the code. 
The example code serial writes 0-9 to demonstrate the support of *printf*. Additionally, the line:
```c
Serial.write(Serial.read());
```
commands the Serial Monitor to echo back typed messages, as shown below.  

![](/lab1/serial.png)
### 4 Temperature Sensor

The *Example2_analogRead* code was compiled and uploaded. The serial output is shown below, where blowing or pressing on the sensor caused the reading to jump.

![](/lab1/temp.webm)

<video src="/lab1/temp.webm" muted loop autoplay></video>

### 5 Microphone

The *Example1_MicrophoneOutput* code was compiled and uploaded. I monitored the output of the loudest frequency, which jumped when I spoke into the Artemis or lightly tapped it against a surface.

<video src="/lab1/microphone.webm" muted loop autoplay width="400"></video>

### Additional Task: Tuner

The previous *Example1_MicrophoneOutput* template was used as a base to create a simplified electronic tuner. 
My tuner detects three different octaves of A: A4 (440 Hz), A5 (880 Hz), and A6 (1760 Hz). The modified *printLoudest* function outputs to serial if a note is detected within the three defined frequencies (within a 10 hz error range).   

```c
double a4 = 440;
double a5 = 880;
double a6 = 1760;
double error = 10;

if (abs(ui32LoudestFrequency - a4) < error){
Serial.printf("Note found: A4!");
} else if (abs(ui32LoudestFrequency - a5) < error){
Serial.printf("Note found: A5!");
} else if (abs(ui32LoudestFrequency - a6) < error){
Serial.printf("Note found: A6!");
}

Serial.printf("\n");

}
```
This interactive table from [muted.io](https://muted.io/note-frequencies/) was used to select and play specific frequencies.

Through testing, I found that the microphone could accurately detect frequencies until ~250 Hz, at which playing any lower frequency would output erroneously higher frequencies. Additionally, false positives due to noise occurred.

The video below demonstrates the successful detection of each of the three notes.

<video src="/lab1/tuner.mp4" loop controls></video>