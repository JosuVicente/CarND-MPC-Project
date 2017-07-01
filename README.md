# Nonlinear Model Predictive Control with actuator delays.
Self-Driving Car Engineer Nanodegree Program

---

<p align="center">
<a href="https://www.youtube.com/embed/M-ln2ERYxz0" target="_blank"><img src="https://img.youtube.com/vi/M-ln2ERYxz0/0.jpg" 
alt="NMPC" width="480" height="360" border="10" /></a>
</p>
<p align="center">
<a href="https://www.youtube.com/embed/M-ln2ERYxz0" target="_blank">https://www.youtube.com/watch?v=M-ln2ERYxz0</a>
</p>

## Description
The purpose of this project is to guide a car through a track in a simulator by controlling its steering and speed. To accomplish this task a MPC is used to calculate those values and pass them back. Values provided by the simulator with the state of the car are used to calculate the steering and speed. In addition to the steering and speed values a list of coordinates with the waypoints along the trajectory path and another list with the coordinates of the calculated trajectory are passed back to the simulator to be displayed.

## Vehicle Model
The model used for the vehicle in the simulator is a kinematic bicycle model which is a model which by neglecting some dynamical effects simplifies the jobs althogugh it sacrifices accuracy. The following equations are used on this model:

##### Position (x, y)    
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      
##### Orientation (psi)    
      // psi_[t+1] = psi[t] - v[t] / Lf * delta[t] * dt
      
##### Velocity (v)    
      // v_[t+1] = v[t] + a[t] * dt
      
##### Cross Track Error (cte)    
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      
##### Orientation Error (epsi)    
      // epsi[t+1] = psi[t] - psides[t] - v[t] * delta[t] / Lf * dt

`delta` is the steering angle actuator and `a` the acceleration actuator.

## Configuration variables
##### Lf
Is the distance between the center of gravity of the vehicle and the front wheels. It's set to `2.67`.

##### N
Timestep lenght. It's set to `12`.

##### dt
Timestep frequency. It's set to `0.05s`. With the value of N this means we're going to predict the trajectory for `0.6s`.

##### ref_v
Is the reference velocity of the vehicle and it's set to `65mph`.

##### latency_units
The latency is `100ms` so the value of latency units it's calculated like this: `(int) 0.1 / dt`. With the values used this is `2`.

Other values were tried for N and dt but found these ones to be the ones that let the vehicle drive as fast as 65mph without going out of track based on the model developed. As mentioned above the prediction horizon is `0.6s`. Short prediction horizons are more responsive but long ones are more accurate so it's a trade off.

## Process
With the values received by the simulator first thing is to transform the map coordinates of the waypoints provided into map coordinates:
``` 
//        auto waypoints = Eigen::MatrixXd(2, ptsx.size());
//		  for (auto i = 0; i<ptsx.size(); ++i) {
//			  waypoints(0, i) = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
//			  waypoints(1, i) = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);
//		  }
```

After that we calculate the cte and the epsi by fitting a 3rd order polynomial from the waypoints:
``` 
//        auto coeffs = polyfit(waypoints.row(0), waypoints.row(1), 3);
//		  double cte = polyeval(coeffs, 0); 
//		  double epsi = -atan(coeffs[1]);  
```

And then initialize the state and pass it along with the polynomial to the Solve function that will return the steering and speed that we normalize to send it back to the simulator:
``` 
//        state << 0, 0, 0, v, cte, epsi;
//		  auto vars = mpc.Solve(state, coeffs);
//		  double steer_value = vars[0] / (deg2rad(25));
//		  double throttle_value = vars[1];
```
The solve function given the initial state and the third order polynomial will calculate the values to pass to the actuators from those that minimize the cost function. In the solve function the cost is calculated like this:
``` 
//        state << 0, 0, 0, v, cte, epsi;
//        for (unsigned int t = 0; t < N; t++) {
//        		fg[0] += CppAD::pow(vars[cte_start + t], 2);
//        		fg[0] += 10 * CppAD::pow(vars[epsi_start + t], 2);
//        		fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
//        }
//        
//        for (unsigned int t = 0; t < N - 1; t++) {
//        		fg[0] += CppAD::pow(vars[delta_start + t], 2);
//        		fg[0] += CppAD::pow(vars[a_start + t], 2);
//        }
//        
//        for (unsigned int t = 0; t < N - 2; t++) {
//        		fg[0] += 10000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
//        		fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
//        }
```

And the constraints:
``` 
//      for (unsigned int t = 1; t < N; t++) {
//          AD<double> x1 = vars[x_start + t];
//      	AD<double> y1 = vars[y_start + t];
//      	AD<double> psi1 = vars[psi_start + t];
//      	AD<double> v1 = vars[v_start + t];
//      	AD<double> cte1 = vars[cte_start + t];
//      	AD<double> epsi1 = vars[epsi_start + t];
//      	AD<double> x0 = vars[x_start + t - 1];
//      	AD<double> y0 = vars[y_start + t - 1];
//      	AD<double> psi0 = vars[psi_start + t - 1];
//      	AD<double> v0 = vars[v_start + t - 1];
//      	AD<double> cte0 = vars[cte_start + t - 1];
//      	AD<double> epsi0 = vars[epsi_start + t - 1];
//      	AD<double> delta0 = vars[delta_start + t - 1];
//      	AD<double> a0 = vars[a_start + t - 1];
//      	AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
//      	AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
//      
//      	fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
//      	fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
//      	fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
//      	fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
//      	fg[1 + cte_start + t] = cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0) * dt));
//      	fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 + v0 / Lf * delta0 * dt);
//      }
```

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
