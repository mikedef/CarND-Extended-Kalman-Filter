# Extended Kalman Filter Project 

## Michael DeFilippo

#### Please see my [project code](https://github.com/mikedef/CarND-Extended-Kalman-Filter/tree/master/src) for any questions regarding implementation.
---

** Extended Kalman Filter (EKF) Project**
---
The goals/steps of this project are the following:
- Install and build EKF project starter code alonge with the project [simulator](https://github.com/udacity/self-driving-car-sim/releases). 
- Implement an EKF in C++ based off the previous lessons using the project starter code and simulator. 
- Create an algorithm that can accomplish better accuracy than [.11, .11, 0.52, 0.52]. 

## Important Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Accuracy
#### The output coordinates px, py, vx, vy must have an RMSE <= [.11, .11, 0.52, 0.52] when using the input file provided. 
![alt text](EKF_RadarPlusLidar_ZoomedOut.png)

The accuracy for px,py,vx,vy respectivily are as shown in the above image of 0.097,0.085,0.451,0.44. This is less than the required accuracy foe the EKF algorithm. 

## Follows the Correct Algorithm
#### Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
The standard Kalman and Extended Kalman filter implimentations can be found in the following files of [kalman_filter.cpp](https://github.com/mikedef/CarND-Extended-Kalman-Filter/tree/master/src) and [FusionEKF.cpp](https://github.com/mikedef/CarND-Extended-Kalman-Filter/tree/master/src). 

#### Kalman Filter algorithm handles the first measurements appropriately.

The first measurements are handled in FusionEKF.cpp. The program is robust enough to take in either a Laser or Radar measurement first and then initialize the EKF approriatly. 

#### Kalman Filter algorithm first predicts then updates.

In FusionEKF I first predict the state of the vehicle, then I update the state based on the incoming Laser or Radar measuremtents. 

#### Kalman Filter can handle radar and lidar measurements.

The EKF can handle both Radar and Lidar measurements. Essentially this is handled by either sending the Lidar measurements directily to a Kalman filter, or sending the Radar measurements to an EKF after converting the coordinats from polar to cartiesion. 

## What's the difference between the combined radar/lidar EKF versus pure radar EKF or a pure lidar EKF?
Below I compare the accuracy of running an EKF with both radar and lidar measurements, versus having just radar measurements, or just lidar measurements. It is clear below that the more data (radar and lidar) will provide a more accurate state estimation.

![alt text](EKF_RadarPlusLidar.png)
EKF with Radar and Lidar Measurements

![alt text](EKF_NoLidar.png) 
EKF with Radar and No Lidar Measurements

![alt text](KF_NoRadar.png)
EKF with Lidar and No Radar Measurements
