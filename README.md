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

The output coordinates px, py, vx, vy must have an RMSE <= [.11, .11, 0.52, 0.52] when using the input file provided. 

## Follows the Correct Algorithm
#### Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

#### Kalman Filter algorithm handles the first measurements appropriately.

#### Kalman Filter algorithm first predicts then updates.

#### Kalman Filter can handle radar and lidar measurements.


