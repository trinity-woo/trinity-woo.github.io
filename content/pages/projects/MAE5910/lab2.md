+++
title = "Lab 2: Inertial Measurement Unit"
date = 2026-02-03
template = "page.html"
+++

Adding the IMU, storing data, and setting up the RC car! <!-- more -->

## Lab 2

### Prelab

### IMU Setup

(picture of IMU connections on phone)

The *Example1_Basics* code was run, and the IMU data was succcesfully output to the Serial Monitor:

<video src="/lab2/imu_example.webm" muted loop autoplay></video>

```c++
#define AD0_VAL 1
```

This value represents the address of the I<sup>2</sup>C, as shown in the datasheet, and it was set to 1, the default value. Soldering or closing the ADR jumper would change the value to 0. 

The sensor values were read as I accelerated and rotated the board in each axis direction.

The accelerometer values increased when accelerating in the x and y directions. The z value remained constant due to gravity.

![](/lab2/acc_x.png)
![](/lab2/acc_y.png)

Additionally, when the board was flipped, the z direction, gravity, also flipped in value.

![](/lab2/acc_flipped.png)

The gyrometer values increased when rotating in the x, y, and z directions.

![](/lab2/gyro_x.png)
![](/lab2/gyro_y.png)
![](/lab2/gyro_z.png)

To indicate that the board is running, the Artemis' LED was blinked 3 times in the *void setup()* function:

```c++
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < 2; i++) {

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);

  }
```

### Sampling IMU Data

To start or stop IMU data recording, the *COLLECT_IMU* case sets the global variable *collectIMU* to 1 or 0 depending on the sent command. 

```c++
case COLLECT_IMU: 
    {

      int collect;

      success = robot_cmd.get_next_value(collect); 
        if (!success)
          return;

      if (collect) {
        collectIMU = 1;
      } else {
        collectIMU = 0;
      }

      break;
    }
```

The main loop continuously runs the *get_IMU()* helper function, which checks if the IMU data is ready to be collected, and stores it if so. 

```c++
if (collectIMU && myICM.dataReady() && (curIMU < (maxSize - 3))) {
      get_IMU();
  }
```

I decided to store the data in separate arrays for the accelerometer and gyroscope, as it would be easier to parse and analyze the data. Additionally, they are stored as floats since converting the raw data to pitch, roll, and yaw will output decimal values. Although a double would be more precise, it would consume twice as much memory, and the float precision is sufficient for our purposes.

```c++
void get_IMU() { 
  float t;

    myICM.getAGMT();  
    t = millis();    
    timeArray[curIMU] = t;
    accArray[curIMU] = (myICM.accX());
    accArray[curIMU+1] = (myICM.accY());
    accArray[curIMU+2] = (myICM.accZ());

    gyroArray[curIMU] = (myICM.gyrX());
    gyroArray[curIMU+1] = (myICM.gyrY());
    gyroArray[curIMU+2] = (myICM.gyrZ());

    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append(timeArray[curIMU]);
    tx_estring_value.append("; ");
    tx_estring_value.append("Acc:");
    tx_estring_value.append(accArray[curIMU]);
    tx_estring_value.append(", ");
    tx_estring_value.append(accArray[curIMU+1]);
    tx_estring_value.append(", ");
    tx_estring_value.append(accArray[curIMU+2]);
    tx_estring_value.append("; ");

    tx_estring_value.append("Gyro:");
    tx_estring_value.append(gyroArray[curIMU]);
    tx_estring_value.append(", ");
    tx_estring_value.append(gyroArray[curIMU+1]);
    tx_estring_value.append(", ");
    tx_estring_value.append(gyroArray[curIMU+2]);
    tx_estring_value.append(";");

    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    curIMU += 3;
}
```

By setting up a simple notification handler that prints the received message, and sending the command to collect data for 5 seconds:

```python
ble.send_command((CMD.COLLECT_IMU), 1)
time.sleep(5)
ble.send_command((CMD.COLLECT_IMU), 0)
```

The following IMU data is printed:

<video src="/lab2/5seconddata.webm" muted loop autoplay></video>

We observe that the IMU data is produced every ~10 ms. Each loop of data sent contains 7 float values (or 28 bytes), we can store around 12500 values from the 350000 bytes of dynamic memory, or ~125 seconds of data collection.

By running a Serial.print command through the main loop:

![](/lab2/mainloop.png)

We observe that the main loop is run every ~1.3 ms, which is 10x faster than the rate of the IMU.


### Accelerometer

The following equations were used to convert accelerometer data into pitch and roll in degrees:

```c++
theta = atan2(a_x, a_z) * 180/math.pi
phi = atan2(a_y, a_z) * 180/math.pi
```



Image of output at {-90, 0, 90} degrees for pitch and roll (include equations)
Accelerometer accuracy discussion
Noise in the frequency spectrum analysis
Include graphs for your fourier transform
Discuss the results

### RC Car

!!videos on ipad

extremely sensitive to controls, difficult to turn precisely (even one tap makes it turn 90 degrees), accelerates quickly to max speed
easily flips over when running into an obstacle