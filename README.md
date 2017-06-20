# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
# MPC Project

## Introduction
In this project we use MPC (Model Predictive Control) controller to make the car drive itself in the simulator. There's simulated 100 millisecond latency between the actuation and the vehicle response. 

## How MPC controller works in plain English
The idea behind MPC is simple. The controller is given a **path** to follow, which is planned out by the path planner (which would be another topic). The path is represented by **waypoints**. So the first job of the MPC is to **connect the dots using a polynomial** so that it knows where the car should be for any given time. Once the target is clear, MPC then has to determine a **window of prediction** where MPC predicts where the car is going to be sometime in the future. We can think of this path within this window as another function, and the **distance** between this function and the polynomial we talked about earlier as the **cost**, which we want to minimize. Given the current state and actuation, we know that we can use Newton's laws of motion to calculate where the car's going to be for any given time in the future, which sounds a bit too good to be true. **All of this is legit, only if all of our information is legit**. However, in the real world, data is noisy and also limited, because anything can happen on the road and we can't account for all that. We know that actuation (steering and thrust) is something we can control, so they are the parameters of our function of predicted path. Since we have determined our cost function as the distance between the said two functions, we can solve this non-linear optimization problem to find the best actuation within this window. We can choose this window to be very small so that this fortune-telling thing isn't too far off. However, no matter how small this window is, we know two things: 
1. The further into the future, the less accurate we can predict. 
2. We are really only interested in what we need to do NOW.

So although we can optimize for actuation for any given time within this window, the most accurate and relevant information is the actuation of now. We can discard the rest and get them next time when we need to determine our actuation using the same method. 

Picking the size of this window is tricky. You don't want it to be too large, and you don't want it to be too small. If it is too large, your prediction is basically nonsense and all this optimization is just optimizing nonsense. But if you make it too small, then you can't account for some sharp turns if you're going really fast, because you're too short-sighted. Another thing to think about is how often you want to change your actuation. If you do it too often, you're wasting your computing power because there's latency in the system so basically within that delta t the actuation is all the same. But if you only make the change once in a while, you'll be in the water.

## How MPC works in pseudo code with annotation
Here is the MPC algorithm from Udacity SDCND:

Setup:

1. **Define the length of the trajectory, `N`, and duration of each timestep, `dt`**. This length of trajectory is just the window we talked about earlier, and `dt` is the reciprocal of "how often we change actuation". The length of the trajectory, let's called it `W` is the multiplication of `N` and `dt`.
2. **Define vehicle dynamics and actuator limitations along with other constraints.** This is the governing equation we talked about earlier. The motion of the car is governed by Newton's law of motion, so if your current speed and location and orientation are determined, and you know your thrust, and steering, then the status of the car in the next timestamp is determined. We call this the constraints. There are also some other constraints we need to account for like the range of the thrust and steering. 
3. **Define the cost function.** We talked about this earlier. The cost is bad, so we want it to be small. Here we know the target is to follow that path laid out in front of us, so any deviation is bad. 

Loop:

1. **We pass the current state as the initial state to the model predictive controller.** This is obvious. No comment.
2. **We call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function. The solver we'll use is called *Ipopt*.** Ipopt is a non-linear solver that can solve problems of optimization with constraints. Good stuff.
3. **We apply the first control input to the vehicle.** This is just what I said earlier. The rest of the controls are kind of like fortune-telling. We took the time to calculate all of them but don't feel bad discarding them, we'll get something better in the next timestamp.
4. **Back to 1.** That's why we call it a loop.

## Timestep length and frequency (`N` and `dt`)
I think it's obvious that 0.1 is a good choice for `dt` since that's the latency, so anything smaller than that doesn't make sense. On the other hand, in the real, continuous world, `dt` is essentially 0, and we want our computer to be as close to this as possible without comprising the performance, so I think 0.1 is good.

Once `dt` is determined, we just need to adjust `N` so that `N * dt` is reasonable. I started with `N = 10`, but the car drives in a way that's very close to what I would called dangerous, so I tried to twiddle it manually to see if there's anything better in the vicinity. When `N = 8`, the car is driving more radically as it would fail to see harsh turns. At `N = 15`, the car would think the world is too dangerous and won't go anywhere in the sharp turns. In the end, I settled for `N = 12`.

## Latency
One way I can think of to account for latency is you try to predict when the car would be when the response comes. We know the latency is 100 milliseconds so that means the response would come in 100 milliseconds each time you change your actuation. Since we know that that's when the response would come, we know that the status of the car at that time would be different then that of now, so we can account for that using Newton's law of motion again. So here's how I calculated the delay status:

```cpp
double delta_t = 0.1;
double current_px = v * delta_t;
double current_py = 0;
double current_psi = (v * (-delta) / Lf) * delta_t;
double current_v = v + a * delta_t;
double current_cte = cte + v * sin(epsi) * delta_t;
double current_epsi = epsi + (v * (-delta) / Lf) * delta_t;
```

Theoretically this should work, but well, for some reason it doesn't. While passing the current status works pretty well, with this delayed implementation the car would go haywire at harsh turns. I think there might be some bugs in my optimizer or FG but I couldn't find it. So in the end I had go back using current status, and slow down the car so the this delay is negligible. 