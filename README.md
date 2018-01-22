# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

This project contains an implementation of a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

The code evaluated using the simulator. Resulted error in position and velocity of the moviding object calculated as [RMSE](https://en.wikipedia.org/wiki/Root-mean-square_deviation) and provided in simulator GUI and as a summary in console output.

# Results discussion
RMSE metric |   px    | py | vx | vy
----------- | ------- |  ------- | ------- | ------- 
Base                      |libapp.a             |✓|✓|[Lars Pontoppidan](https://github.com/Larpon)
AdMob                     |libadmob.a           |✓|✓|[Lars Pontoppidan](https://github.com/Larpon)
Analytics                 |libanalytics.a       |✓|✓|[Lars Pontoppidan](https://github.com/Larpon)
~~Authentication~~	          |~~libauth.a~~          |in progress |:wrench: |[Isy](https://github.com/isipisi281)
~~Realtime Database~~	        |~~libdatabase.a~~      |in progress |:wrench: |[greenfield932](https://github.com/greenfield932)
~~Invites and Dynamic Links~~	|~~libinvites.a~~       | | |
Cloud Messaging	          |libmessaging.a     |✓|✓|[Isy](https://github.com/isipisi281)
Remote Config             |libremote_config.a   |✓|✓|[greenfield932](https://github.com/greenfield932)
~~Storage~~	                  |~~libstorage.a~~       | | |


Both
	Mean: 0.101144 0.0897438 0.722948 0.537216
	Min:  0.0832621 0.0688891 0.451267 0.417613
	Max:  0.242874 0.109475 4.29483 1.75481

LIDAR only
    Mean: 0.155385 0.114369 0.895262 0.534863
	Min:  0.107757 0.0196827 0.63679 0.00127057
	Max:  0.437515 0.131132 5.19984 0.897538


RADAR only
	Mean: 1.83898 1.64836 1.8947 1.71698
	Min:  0.172504 0.0876373 0.626301 0.676337
	Max:  11.5726 8.39656 9.28523 7.03935

# Installation and building

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---


## Running

1. Run simulator
2. Run the program: `./ExtendedKF `
3. Start simulation

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

