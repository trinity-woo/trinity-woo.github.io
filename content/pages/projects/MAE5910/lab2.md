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

After receiving the IMU data over bluetooth, a notification handler is set up to extract the accelerometer data. The following equations from *Sensors II Lecture* were used to convert accelerometer data into pitch and roll in degrees:

```python
theta = math.atan2(accX, accZ) * 180/math.pi
phi = math.atan2(accY, accZ) * 180/math.pi
```


To determine the accuracy of the accelerometer, the output at either end of the pitch and roll ranges is determined. A flat edge was used to ensure perfect angles.

The individual output of the pitch and roll at 90 and -90 degrees is shown:

![](/lab2/pitch90.png) ![](/lab2/pitch-90.png) ![](/lab2/roll90.png) ![](/lab2/roll-90.png)

With a two-point calibration, we calculate the conversion factor to be 90/88.2 or ~1.02. The completed notification handler is shown below:

```python
def notification_handler(uuid, byte_array):

    m = ble.bytearray_to_string(byte_array)

    extract = m.strip().split(';')

    time = extract[0].split(':')[1]

    acc_values = extract[1].split(':')[1].split(',')
    accX, accY, accZ = [float(v.strip()) for v in acc_values]
    

    theta = math.atan2(accX, accZ) * 180/math.pi
    phi = math.atan2(accY, accZ) * 180/math.pi


    print("theta: ", theta*conversion, "phi: ", phi*conversion)
```

The output of moving the IMU -90 degrees in pitch and 90 degrees in roll is shown: 

![](/lab2/-9090.png)


Next, we analyze the noise produced by the accelerometer data and perform a Fourier Transform. First, the IMU data lying flat on the table is graphed:

![](/lab2/imu_graph.png)

Then, a Fourier transform is applied and the results are graphed:

```python
N = int(wait * sample_rate)

theta = np.array(theta_list)
t = np.array(times)

freq_data = fft(theta_list)
y = 2/N * np.abs (freq_data [0:int (N/2)])
```

![](/lab2/ftpitch.png)
![](/lab2/ftroll.png)

From observation, the cutoff frequency $f\_c$ is picked to be 3 hz to mitigate the beginning peaks. A too high cutoff frequency reduces more noise, but may block out wanted signals in which the car experiences sharp changes in angle or turns. To apply a low pass filter, the $\alpha$ value is calculated from the equations, where T represents the inverse of the sampling rate: 

$$
\alpha = \frac{T + RC}{T}
$$

$$
f\_c = \frac{1}{2\pi RC}
$$

$\alpha$ is found to be 0.2498.

The new low pass filter values are calculated and graphed:

```python
theta_lpf = np.zeros_like(theta_raw)
phi_lpf   = np.zeros_like(phi_raw)

theta_lpf[0] = theta_raw[0]
phi_lpf[0]   = phi_raw[0]

for n in range(1, len(theta_raw)):
    theta_lpf[n] = alpha * theta_raw[n] + (1 - alpha) * theta_lpf[n-1]
    phi_lpf[n]   = alpha * phi_raw[n]   + (1 - alpha) * phi_lpf[n-1]
```

![](/lab2/lpf.png)

The low pass filter drastically reduces the noise in the pitch and roll values.

### RC Car

!!videos on ipad

extremely sensitive to controls, difficult to turn precisely (even one tap makes it turn 90 degrees), accelerates quickly to max speed
easily flips over when running into an obstacle

### Collaborations

website for sampling data (how to collect the accelerometer and gyroscope data)
chat for setting up notification handler, extracting and plotting the data

https://www.alphabold.com/fourier-transform-in-python-vibration-analysis/ for plotting Fourier transform