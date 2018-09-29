# CarND-Controls-MPC
This projectuses Model Predictive Control(mpc) to control a vehicle running in Udacity Simulator. It takes into account vehicle motion equations to predict and control the movement of vehicle.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Rubric Points Discussion
* **Model:** 
The project utilizes vehicle kinematics model but does not consider tyre dynamics. Equation for the model state transition are as follows:
* **State Model:**
1. x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
2. y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
3. psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
4. v_[t+1] = v[t] + a[t] * dt
5. cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
6. epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
Where,
   x , y : Denotes vehicle position
   psi   : heading angle
   v     : vehicle velocity
   cte   : cross track error(lateral distance of vehicle from center line in road)
   epsi  : orientation error
   delta : steering angle
   a     : acceleration
   Lf    : Distance of front of vehicle to the center of gravity of vehicle
Here, "delta" and "a" are the actuator control. The goal of MPC is to find "delta" & "a" value in a way which minimize the cost calculated using various factors such as:  
1. squared sum of cte, epsi, velocity w.r.t reference state.
2. squared sum of delta and a to minimize the use of actuators.
3. squared sum of difference between consecutive actuator values to minimize the sharp change of the values.

The weight of the above cost factor are tuned manually to find the values which help the vehicle to maintain the track. These help the vehicle to have smooth transition of steering angle rather than sharp change.

Due to steering angle sign used in unity simulator is opposite to real world steering angle sign for moving in left/ right direction, the delta value given by the websocket is negated.

* **Timestep Length and Elapsed Duration (N & dt):**
These are used for predicting horizon.
The final value of N (timestep length) and dt (elapsed duration between timesteps) used are, N=10, dt=0.1. These values are again chosen manually. A bigger value of N will increase the calculation time and in turn impact real time performance. This makes the vehicle's movement erratic specially on the turns. The values tested were N=15/20 and corresponding dt value used are 0.05/0.09. With N=10 and dt=0.1 gives  smooth vehicle control in the turnings also.

* **Polynomial Fitting and MPC Preprocessing:**
3 rd order Polynomial has been used to calculate the coefficient using the waypoint given. Waypoins are first converted to vehicle co-ordinate before processing.  It involves rotation and translation mechanism to align the vehicle x axis as a horizontal line. The coefficient thus generatedare then used for calculating cte and epsi. These are also used by IpOpt solver to find the actuator values by minimizing cost.In file main.cpp, Line 102 is the starting  point for the axis translation.

* **Model Predictive Control with Latency:**
The latency of 100 millisecond is taken into consideration. The state variables are calculated after 100 millisecond passed using the initial state values and the final state value thus obtained is used for mpc. In main.cpp, line 129, these latency calculations are done.

