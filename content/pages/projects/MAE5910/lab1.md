+++
title = "Lab 1: Artemis and Bluetooth"
date = 2026-01-23
template = "page.html"
+++

Setting up the Artemis board and sending bluetooth data between the computer and the robot. <!-- more -->

## Lab 1A
### Prelab
This section's setup included updating the ArduinoIDE and installing the custom Sparkfun Apollo3 board definition for development with the Artemis. 

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

```c++
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


## Lab 1B
### Prelab

This section establishes Bluetooth Low Energy (BLE) communication between the Artemis and the computer.
A virtual environment was set up and activated, where the Jupyter server was ran. Python 3.13.11 is used, and the following packages are installed:

```python
 pip install numpy pyyaml colorama nest_asyncio bleak jupyterlab
```

The provided codebase includes scripts for the Artemis (Arduino/C++) and the computer (Python).

In addition to the main Arduino script, there are three class definitions: for sending/receiving data; editing character arrays; and extracting command values sent by the computer.

The Python side includes; *ble.py* to define Bluetooth and data handling functions; a script to define command types sent to the robot; *connections.yaml* to define addresses and UUIDs; a script that formats logs and outputs to a directory. We also use a ipynb (Jupyter Notebook) to write commands, and send/receive Artemis data.

BLE communication follows a client-server model, where central devices (in this case, the computer) read or write data from a peripheral device (the Artemis board). 

To establish connection between my specific Artemis board and the computer, the Artemis' MAC address and a randomly generated UUID are defined in the Python *connections.yaml* script. The UUID defined in the Arduino script must match. The Artemis advertises with its MAC address, and the computer connects by searching for the address and UUID.

![](/lab1/macaddress.png)

![](/lab1/connected.png)

### 1 Echo

The ECHO command in the Arduino code was modified to return the string send by the computer.

```c++
case ECHO:

      char char_arr[MAX_MSG_SIZE];

      // Extract the next value from the command string as a character array
      success = robot_cmd.get_next_value(char_arr);
      if (!success)
        return;

      tx_characteristic_string.writeValue(char_arr);

      break;
```

![](/lab1/woop.png)

### 2 Send Three Floats

The SEND_THREE_FLOATS command was modified:

```c++
case SEND_THREE_FLOATS:

      float int_c, int_d, int_e;

      success = robot_cmd.get_next_value(int_c);
      if (!success)
        return;

      success = robot_cmd.get_next_value(int_d);
      if (!success)
        return;

      success = robot_cmd.get_next_value(int_e);
      if (!success)
        return;

      Serial.print("Three Integers: ");
      Serial.print(int_c);
      Serial.print(", ");
      Serial.print(int_d);
      Serial.print(", ");
      Serial.println(int_e);

      break;
```

When running the following code cell in Jupyter, separating each command with |:

```python
ble.send_command(CMD.SEND_THREE_FLOATS, "1|2|3")
ble.send_command(CMD.SEND_THREE_FLOATS, "0|6|2")
```

The Artemis receives and outputs the floats. 

![](/lab1/3int.png)

### 3 Get Time

The built-in *millis* function is utilized to keep track of the current program runtime. 

```c++
case GET_TIME_MILLIS:

      unsigned long t;

      t = millis();

      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(String(t).c_str());
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      break;
```