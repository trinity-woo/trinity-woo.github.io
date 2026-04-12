+++
title = "Lab 8: Stunts"
date = 2026-04-05
template = "page.html"
+++

Drifting! <!-- more -->

### Flip Setup

For the flip stunt, the car will start at some distance from the wall to reach maximum speed, stop 1 ft from the wall, flip, and drive back to the starting line.

Instead of using the Kalman Filter values to estimate the distance to the wall, I decided to manually set the time the motors drive forwards and backwards. This open-loop control is not robust, but is simple and does rely on sensor data.

I wrote a new *FLIP* command that sets the *flip* boolean and the time the car will drive forwards and backwards.

```c++

bool flip = 0;
int flipForward = 2000;
int flipBackward = 2000;

...

    case FLIP:
      {

        int collect, forward, back;
        success = robot_cmd.get_next_value(collect);
        if (!success)
          return;

        success = robot_cmd.get_next_value(forward);
        if (!success)
          return;

        success = robot_cmd.get_next_value(back);
        if (!success)
          return;

        if (collect) {
          flip = 1;
        } else {
          flip = 0;
        }

        flipForward = forward;
        flipBackward = back;

        break;
      }
```

Then, in the main loop, the *flip* boolean is checked to start the flip sequence, setting the car to max speed for the set time, and then suddenly reversing direction, causing the car to flip and return to start.

```c++
...
      if (flip) {

        abs_pwm = 255*Sp;
        float start = millis();

        while (millis() - start < flipForward) {
          forward(abs_pwm);
          get_PWM();
          if (distanceSensorFront.checkForDataReady() && distanceSensorSide.checkForDataReady()) {
            get_TOF();
          } 
        }

        stop();

        while (millis() - start < (flipForward + flipBackward)) {
          back(abs_pwm);
          get_PWM();
          if (distanceSensorFront.checkForDataReady() && distanceSensorSide.checkForDataReady()) {
            get_TOF();
          } 
        }

        flip = 0; // finished stunt
        stop();

        if (send) {
          send_data();
        }
    }
...
```

Despite adding more weight to the front of the car and increasing the speed, the car was only able to get off the ground for a few inches and could not complete the flip after many attempts. Therefore, I ~gave up and~ decided to switch to the drift stunt, which follows a similar code architecture. 

### Drift Setup

For the drift stunt, the car will start at some distance from the wall to reach maximum speed, and turn 180 degrees within 3 ft. Open-loop control is implemented.

```c++
bool drift = 0;
int driftForward = 1000;
int driftTurn = 100;
```

```c++
...
      if (drift) {

        abs_pwm = 255 * Sp;
        float start = millis();

        while (millis() - start < driftForward) {
          forward(abs_pwm);
          get_PWM();
          if (distanceSensorFront.checkForDataReady() && distanceSensorSide.checkForDataReady()) {
            get_TOF();
          }
        }

        stop();
        delay(10);

        while (millis() - start < (driftForward + driftTurn)) {
          left(abs_pwm);
          get_PWM();
          if (distanceSensorFront.checkForDataReady() && distanceSensorSide.checkForDataReady()) {
            get_TOF();
          }
        }
        drift = 0;  // finished stunt
        stop();

        if (send) {
          send_data();
        }
      }
```

I encountered some issues where the wheels would not reverse (to turn) after moving forward, in which the *stop()*, *delay(10)* helped. Speeding up the car and increasing the starting distance was also essential to overcome friction. The left motor PWMs were multiplied by a calibration factor (as written in Lab 4), with a forward calibration of 1.12 and a turn calibration of 1.3.

Due to the calibration factor, there was a limit in the maximum PWM, since at 100%, there would be no room for the left motor PWMs to be greater than the 255.

After trial and error, I determined the best speed factor, forward time, and turn time:

```python
Sp = 0.7
forward = 780
turn = 250
```

With adding up the forward and turn times, the stunt takes 1030 milliseconds, or just over 1 second.


### Stunts!

The sensor data and PWM values are plotted with each trial. Since the Kalman Filter was not activated, the ToF sensor values are sparse. It is also possible that the *drift()* blocked the sensors from properly checking for data.

Three successful trials are shown below:

<video src="/lab8/trial2.MOV" controls></video>
![](/lab8/drift1.png)
![](/lab8/drift2.png)

<video src="/lab8/trial3.MOV" controls></video>
![](/lab8/drift5.png)
![](/lab8/drift6.png)

<video src="/lab8/trial4.MOV" controls></video>
![](/lab8/drift7.png)
![](/lab8/drift8.png)


### Collaborations

[Evan's site](https://evnleong.github.io/MAE4190/lab7.html) was referenced for the idea of having open-loop forward and backward times.