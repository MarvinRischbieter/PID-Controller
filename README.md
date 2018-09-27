# Introduction
This is Project 4 of Term 2 of the Udacity Self-Driving Car Nanodegree program. The goal is to implement PID controller in C++ to drive the car around the track in the [Simulator](https://github.com/udacity/CarND-PID-Control-Project/releases). For that project the [Starter Code](https://github.com/udacity/CarND-PID-Control-Project) was used.

## Theory of PID controllers

PID stands for Proportional, Integral, Derivative. It is a control loop feedback controller which is widely used in different control systems.

The Error is an input variable for the controller:
_cte = desired_state - measured_state_

With _P_ only, the controller output is proportional to the _cte_. Only the present value of _cte_ is taken into account. With this part of controller, the car will be able to steer in the correct direction.

The Integral term _I_ is the integral of _cte_ over the past time, in order to reduce systematic bias.  In this project I found out that it is not necessary to integrate for all time. It is enough to calculate it over the last _n_ frames.  However, one problem is that left turns are prevalent in the track, which results in a different control behavior. Because of this I impltemented a buffer using the code from [stackoverflow](https://stackoverflow.com/questions/25024012/running-sum-of-the-last-n-integers-in-an-array).

The derivative _D_ part takes into account that the controller output is proportional to the change-rate of _cte_ (derivative). Therefore _D_ is used to lower overshooting and damping oscillations caused by _P_.

Implementation of the PID control is trivial - tuning it is the bigger effort.

## PID Tuning

In the beginning I tuned the controller using [Ziegler–Nichols method](http://staff.guilan.ac.ir/staff/users/chaibakhsh/fckeditor_repo/file/documents/Optimum%20Settings%20for%20Automatic%20Controllers%20(Ziegler%20and%20Nichols,%201942).pdf). 

This required to set _Kd_ and _Ki_ to 0 and gradually increase _Kp_ before the car drives stable and with at least bounded oscillations. 

The value of _Kp_ and the oscillation characteristics are used to tune the PID controller parameters by the mentioned [method](http://www.mstarlabs.com/control/znrule.html), e.g.:
Classic Ziegler-Nichols:
_Kp_ = 0.6 _Ku_
_Ti_ = 0.5 _Tu_    
_Td_ = 0.125 _Tu_

However, after tuning the controller with this method, the car has driven around the track but with a lot of wobbling. Therefore I started tuning parameters manually by trial-and-error process. Actually, it worked even better without the extra complexity of the tuning method (e.g. Zielger). 

This process was used for different speeds, so that different PID parameters were found. Afterthat the results were linearized in order to make the parameters automatically tune with different vehicle velocity variations. Moreover the implementation considers different time intervals between data frames.

## Dependencies

* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this


## Basic Build Instructions

1. Clone this repo.
2. Build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run: `./pid`. 




