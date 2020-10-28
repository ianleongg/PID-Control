# **PID Control** 

##  Implement a PID controller to control a car in a simulator

### Use twiddle algorithm to tune hyperparameters for PID controller 
---

**PID Controller Project**

The goals / steps of this project are the following:
* Implement a PID controller to control a car in [Udacity's simulator](https://github.com/udacity/self-driving-car-sim/releases)
* Implement twiddle algotithm to automatically tune the hyperparameters 
* Complete a lap around the track

[//]: # (Image References)

[image1]: ./Images_forReadMe/pid.gif "pid"
[image2]: ./Images_forReadMe/pid.mp4 "pid"
[image3]: ./Images_forReadMe/pid.png "pid"
[image4]: ./Images_forReadMe/pid-process.png "pid"
[image5]: ./Images_forReadMe/twiddle.png "twiddle"
[image6]: ./Images_forReadMe/twiddle-pseudo.png "twiddle"

---
### README

- A PID controller was implemented to operate the car for a lap of the track which can be found in the [src folder](./src)

- Below is the result of the PID controller in action:

| Single Lap| 
| ------------- | 
| ![alt text][image1]|
|[Full video here](./Images_forReadMe/pid.mp4)|

- The PID implementation is done on [./src/PID.cpp](./src/PID.cpp). The [PID::UpdateError](./src/PID.cpp#L15) method calculates proportional, integral and derivative errors and the [PID::TotalError](./src/PID.cpp#L27) calculates the total error using the appropriate coefficients.

- The simulator sends cross-track error, speed and angle to the PID controller(PID) using [WebSocket](https://en.wikipedia.org/wiki/WebSocket) and it receives the steering angle ([-1, 1] normalized) and the throttle to drive the car. 

- The PID uses the [uWebSockets](https://github.com/uNetworking/uWebSockets) WebSocket implementation.

- The final hyperparameters used:
```
P (Proportional) = 0.143143
I (Integral) = 0.0028836
D (Differential) = 3.1
```

### 1. Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

```
Listening to port 4567
```

Now the PID controller is running and listening on port 4567 for messages from the simulator. Next step is to open Udacity's simulator, you need to go to the Project 4: PID Controller project. Click the "Select" button, and the car starts driving. 

### 2. Dependencies

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


### 3. Discussion

#### PID Implementation

- The proportional portion of the controller tries to steer the car toward the center line (minimize cross track error, CTE). If used along, the car overshoots the central line very easily and go out of the road very quickly. 

- The integral portion tries to eliminate a possible bias on the controlled system that could prevent the error to be eliminated. If used along, it makes the car to go in circles and to adjust for steering drift. 

- The differential portion helps to counteract the proportional trend to overshoot the center line by smoothing the approach to it. 

- CTE value is read from the data message sent by the simulator, and the PID controller updates the error values and predicts the steering angle based on the total error. This predicted steering angle is a correction of the updated error to the desired setpoint based on proportional, integral, and derivative terms (hence PID).

| PID Formula (image from Wikipedia)| 
| ------------- | 
| ![alt text][image3]|

- After the PID calculates the steering angle, the new CTE value is used to start the process of update and prediction again.

| PID Process (image from Wikipedia)| 
| ------------- | 
| ![alt text][image4]|

#### PID Parameters 

For steering:

- The parameters were chosen manually by try and error. 
- First, make sure the car can drive straight with zero as parameters. 
- Then, add the proportional and the car start going on following the road but it starts overshooting go out of it. 
- Then, add the differential to try to overcome the overshooting. 
- The integral part only moved the car out of the road; so, it stayed as zero. 
- After the car drove the track without going out of it, the parameters increased to minimize the average cross-track error on a single track lap. 

- After initializing said trial and error, a twiddle algorithm is further implemented to update the parameters automatically.

For throttle:

- As for throttle, the logic used was when the CTE is high, the car goes slower while the throttle value is inverse to the steering value when CTE is acceptible.

- Throttle logic:
```
    //THROTTLE
          //slow down when cte is high
          if (fabs(cte) > 1.0) {
            throttle_value = 0.3;
          } 
          //else, throttle = inverse of steering, max 100mph
          else {
            throttle_value = fmin(1.0 / fabs(steer_value), 100.0);
            //normalize throttle value to [0.45, 1.0], max 100mph
            throttle_value = ((1.0 - 0.45) * throttle_value) / (100.0 + 0.45);
          }
```

#### Twiddle Algorithm

- The twiddle algorithm vary each of the parameters and measure the resulting difference in error to determine if increasing or decreasing the value was improving the overall CTE of the car's path.

| Twiddle Psuedocode| 
| ------------- | 
| ![alt text][image6]|

The fundamental concept of twiddle is to refine input parameters based on an empirical feedback mechanism.  The algorithm boils down to:
* Record the error before running
* Change the parameter
* Let the system run
* Record the error again
* Compare the two and chose the one with less error

| Example of twiddle benefiting a PID controller| 
| ------------- | 
| ![alt text][image5]|

- When twiddle runs, it updates the PID hyperparameters directly, and has an immediate affect on the car's performance. Changing the values too much can result in the car immediately flying off the track, param deltas would be initialize to **10% of the seed value**. Even though the twiddle algorithm tunes hyperparameters to a smaller range, it allows for dynamic updates while the simulator is running. 

- Twiddle incorporates a tolerance value as the hyperparameters are tuned, so the algorithm will know when it's finished.**0.2** value was used.

### Conclusion

* The vehicle starts slowly but after the 2 second delay, it completes a lap around the track with approximately 25mph. 
* A higher speed would definitely be more ideal, thus a re-visit is necesarry in the future. 

### Additional resources/information

* [More PID explanations](https://www.youtube.com/watch?v=4Y7zG48uHRo&t=31s)
* [More. PID explanations](https://www.wikiwand.com/en/PID_controller#/Derivative_term)
* [More.. PID explanations](http://oa.upm.es/30015/1/INVE_MEM_2013_165545.pdf)

#### Model Predictive Control (MPC)

* [Vision-Based High Speed Driving with a Deep Dynamic Observer by P. Drews, et. al.](https://arxiv.org/abs/1812.02071)

#### Reinforcement Learning-based

* [Reinforcement Learning and Deep Learning based Lateral Control for Autonomous Driving by D. Li, et. al.](https://arxiv.org/abs/1810.12778)

#### Behavioral Cloning

* [ChauffeurNet: Learning to Drive by Imitating the Best and Synthesizing the Worst by M. Bansal, A. Krizhevsky and A. Ogale](https://arxiv.org/abs/1812.03079)




