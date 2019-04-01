# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Implementation

### The model

The model state consists of position coordinates (x,y), velocity (v), heading (psi), cross-track error (cte), and heading error (epsi).  Actuators involve steering angle (delta) and throttle/brake (a).

The update equations for the model are : 

~~~~
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t-1] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t-1] / Lf * dt
~~~~

### TimeStep Length and Frequency

Timestep length (N) is chosen as 10, with timestep frequency (dt) as 0.1.

High values of N lead to computational lag, as the solver tries to optimize for more constraints.  Low values of N lead to poor accuracy, as not enough of the future path is being planned for.  A value of N=15 was tried, and the program was too slow to update the new actuator values to the simulator, and responded to future states that were often drastically different ffrom the states computed on.  Conversely, a low value of N=5 was tried, and the program was only accounting for very near future points, and around turns, heavily oversteered because it was too short-sighted in planning.

Low values for the timestep, dt, lead to more data points N for the same time horizon N(dt), which lead to higher computation complexity and simulator lag time.  High values for dt lead to discretization error. Here, the way I handled latency was predict the state during the lag time, so dt was made equal to the lag time of 0.1 sec, so this was not a free parameter.

### Polynomial Fitting and MPC Preprocessing

A polynomial is fitted to waypoints.  The waypoints are first preprocessed by converting the waypoints from map coordinates to car coordinates.  The main purpose of converting the waypoints to car coordinates is to be able to easily calcuate the cte, or distance from the car to the nearest point on the polynomial.  In map coordinates, the orthogonal distance from the car to the polynomial would need to be found.  In car coordinates, this orthogonal distance is simply the y component of the polynomial on the car's coordinate system.

These transformations of the waypoints from map to car coordinates were found by expressing the basis vectors for the map coordiate system (1,0) and (0,1) in terms of the car coordinates (displacement and then rotation in the opposite direction of the heading), and then generalizing for arbitrary (x_map, y_map) coordinates : 

~~~
x_car =  (x_map - car_position_x) * cos(-psi) - (x_map - car_position_x) * sin(-psi);

y_car = (y_map - car_position_y) * sin(-psi) + (y_map - car_position_y) * cos(-psi);
~~~

### Model Predictive Control (MPC) with Latency

The actuator values for each timestep were optimized using the IPOPT C++ library.  The cost function optimized was a weighted sum of the following squared costs, each specified with the final weight used : 
- cross track error (cte): 10
- heading error (heading-desired heading): 10
- velocity error (max velocity-current velocity, to make sure the car doesn't go to slow): 1
- steering amount (delta, to minimize steering and make sure steering is not sporadic): 10
- acceleration amount (use the smallest amount of acceleration, all other costs being equal): 1
- the product of steering amount and velocity (to minimize speed around corners, and also minimize steering): 10
- gap between successive steering actuations: 10
- gap between successive acceleration actuations: 1

The weight of each cost component was tuned by considering the priority of each component.  Starting with the highest priority component, that component was tuned with all others off.  Then the remaining components were tuned in descending priority with all those of lower priority off, and so on. 

 From reflection of real driving, keeping in-lane is not done by minimizing lane error in the immediate instant in time, but rather over a certain time horizon.  To ensure more priority is given to the future road, the highest priority is given to the steering cost, and similarly, the product of steering and velocity.  Tuning just these two components resulted in the car staying on the road, but with the predicted path not hugging the waypoints well, so cte became the third tuned component.  The fourth tuned component was heading error.  It was found that this component offsetted the oscillations in correcting for cte, but minimizing steering and the product of steering and velocity were more effective in reigning in the oscillations from reducing cte cost.  The fifth tuned component was the successive steering actuations, for smoother steering transitions. The remaining components were adding into the overall cost but untuned with weights of one.

 The above chosen weights lead to the car successfully driving through the track, at a speed of nearly 80mph.

The simulator had a 100ms latency between an issued actuator command and simulator response.  This latency was simulated with a hard-coded sleep duration of 100ms after the actuator command was issued.  Latency is an issue because when the command is received 100ms later, the state of the car could be wildly different from its state when the command was actually issued.  

I solved this latency issue by predicting the state of the car when the simulator would actually receive the command, after the latency period, and then optimizing for the actuator values at this state.  The state of the car after the latency period was predicted using the model update equations discussed previously. 

---

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
