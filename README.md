# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[PID]: ./image/PID.png "PID"
[Twiddle]: ./image/Twiddle.png "Twiddle"
[Kp]: ./image/Kp.png "Kp"
[Ki]: ./image/Ki.png "Ki"
[Kd]: ./image/Kd.png "Kd"
   
### Simulator.
The Term2 Simulator which contains the Path Planning Project can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project, the goal is to implement a PID controller in C++ to maneuver the vehicle around the track.

---

## File Description

|Files|Description|
|:----|:----------|
|`src/main.cpp`|1. Communicates with the Term 2 Simulator receiving data measurements. <br /> 2. Call functions to update the steering angle and throttle.|
|`src/PID.h` <br /> `src/PID.cpp`|Class for PID controller.|
|`src/PIDWithTwiddle.h` <br /> `src/PIDWithTwiddle.cpp`|Class for PID controller with auto tuning using Twiddle algorithm.|
|`src/RunningData.h` <br /> `src/RunningData.cpp`|Class to compute running statistics.|

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Basic Run Instructions

|Command|Description|
|:------|:----------|
|`./pid`|Running with PID parameters suitable for Udacity's workspace.|
|`./pid run speed Kp Ki Kd`|Running with specified parameters (Car speed, PID Kp, PID Ki, PID Kd)|
|`./pid run high_speed high_speed_Kp high_speed_Ki high_speed_Kd low_speed low_speed_Kp low_speed_Ki low_speed_Kd`|Running using two speed mode with specified parameters. When the running root mean square of cross is smaller than 0.8, it runs with high speed with parameters (high_speed high_speed_Kp high_speed_Ki high_speed_Kd). Otherwise, it runs with low speed with parameters (low_speed low_speed_Kp low_speed_Ki low_speed_Kd) |
|`./pid tune speed initial_Kp initial_Ki initial_Kd [initial_Kp_step initial_Ki_step initial_Kd_step]`|Tuning PID with specified parameters (Car speed, Initial PID Kp, Initial PID Ki, Initial PID Kd, Initial PID Kp Step, Initial PID Ki, Initial PID Kd). if initial_Kp_step initial_Ki_step initial_Kd_step are not provided, they are set to initial_Kp/10 initial_Ki/10 initial_Kd/10.|

---
## Data provided from the Simulator to the C++ Program

### Current car status

["cte"] The current cross track error.

["speed"] The current velocity of the car.

["steering_angle"] The current steering angle of the car.

## Data provided from the C++ program to the Simulator

### Car's control

["steering_angle"] The steering angle of the car. It is ranged from -1 to 1.

["throttle"] The throttle of the car. It is ranged from -1 to 1.

---

## Implementation and Reflection

The implemenation and reflection are described in the followings.

### PID Controller

It is implemented as `class PID` in `PID.h` and `PID.cpp`.

![PID]

PID controllers are used to compute the throttle value and the steering angle.

### Paramters in PID

|Parameters|Description|
|:--------:|:-----|
|P|Proportional to the current error.|
|Kp|Weight of P applied to final control.|
|I|Integrals of all the errors over time.|
|Ki|Weight of I applied to final control.|
|D|Rate of the change of the error. |
|Kd|Weight of Kd applied to final control.|

### Tuning steering angle in PID

#### Kp

When Kp increase, the steering angle would response larger.
The car would come to track center faster.
However, the car would easily over-shoot and also lead to oscillation.
Oscillation amplitude increases with the Kp.

Kp cannot be large when the car velocity is too high.
High Kp makes car oscillate even at straight line.
Setting Kd also cannot help much.
The car would get out of the track easily.
At the same time, Low Kp cannot help the car turn in time at sharp turn.
It is difficult to hanle in hight velocity.

![Kp]

#### Ki

Ki is not important in controlling the steering angle.
Even if it is zero, the car can move steadily on the road.
However, it is discovered that the average error would have little biased.
Setting Ki a quite small value can avoid the system biase in long term.
Setting Ki too big would also lead to oscillation easily.

![Ki]

#### Kd

Kd can acts as a damping parameter.
The oscillation amplitude and duration can decrease with Kd.
However, it cannot make the oscillation disappear completely. 

![Kd]

#### Manual Tuning

1. Start with 10 MPH.
2. Increase Kp until the car can respond to the off cetner.
3. Increase Kd until the oscillation is minimized.
4. Repeat Step 2 and Step 3 until the car can run steady on the track.
5. Add a little Ki which does not affect the current performance.
6. Increase the velocity and repeat Step 2 to Step 5.

It is difficult to tune when the car velocity is over 60 MPH.

### Optimization of Parameters in PID using Twiddle algorithm

It is implemented as `class PIDWithTwiddle` in `PIDWithTwiddle.h` and `PIDWithTwiddle.cpp`. It is implemented using finiste state machine. The pseudocode is described in the following figure.

![Twiddle]


### Parameters in my local machine

Try to optimize Kp, Ki, Kd using Twiddle algorithm under different velocities.
Get the root mean square for 2 laps each time and update the parameters.

|Velocity[MPH]|Kp      |Ki             |Kd     |RMS Error|
|:-----------:|:------:|:-------------:|:-----:|:-------:|
|30           |0.350218|0.0000000121714|2.60246|0.165982 |
|40           |0.29    |0.00000001     |2.7    |0.284529 |
|50           |0.143   |0.00000001     |1.31   |0.584251 |
|60           |0.08712 |0.0000000081   |1.17   |1.18434  |

It is difficult to auto tune the parameters over 50 MPH as the car may get out of the track when trying different parameters.

The driving performance (in terms of osciallation and maximum cross track error) may not be very good at some turns while the overall performance is quite good.

### 2 Speed mode

It is implemented in `main.cpp` line 212 - 220.

When the root mean square of the error in the most 10 frames is smaller than or equal to 0.8,
it run with higher speed with a PID controller.
When the root mean square of the error in the most 10 frames is larger than 0.8,
it run with lower speed with another PID controller.

Mostly likely, the low speed mode would be effective when the car is turning.
The high speed mode would be effective when the car moving at straight line.
Therefore, the Kp would be smaller in high speed mode.
It does not need to handle in most turns.

Run the 2 speed mode using the following parameters in my local machine.

|Mode         |Velocity[MPH]|Kp      |Ki         |Kd  |
|:-----------:|:-----------:|:------:|:---------:|:--:|
|Low Velocity |50           |0.143   |0.000000009|1.31|
|High Velocity|70           |0.07    |0.00000001 |1.2 |

### Parameters in the Udacity workspace

The parameters optimized in my local machine cannot be used to Udacity workspace.
Complete tuning using Twiddle in the Udacity workspace is impossible as it takes too much time.
Fine tune the parameters according to the parameters in local machine and the manual tuning method mentioned.

Run the 2 speed mode using the following parameters.

|Mode         |Velocity[MPH]|Kp      |Ki        |Kd |
|:-----------:|:-----------:|:------:|:--------:|:-:|
|Low Velocity |30           |0.2     |0.00000001|2  |
|High Velocity|60           |0.1     |0.00000001|1  |