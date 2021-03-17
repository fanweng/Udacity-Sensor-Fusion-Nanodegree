# Unscented Kalman Filter Project

In this project you will implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements.

<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has its own UKF object generated for it, and will update each individual one during every time step. 

The red spheres above cars represent the `(x,y)` lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The `Z` axis is not taken into account for tracking, so you are only tracking along the `X/Y` axis.

## I. System Preparations

#### Starter code

[https://github.com/udacity/SFND_Unscented_Kalman_Filter](https://github.com/udacity/SFND_Unscented_Kalman_Filter)

#### Build requirements

- cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
- gcc/g++ >= 5.4
    * Linux: gcc/g++ is installed by default on most Linux distros
- PCL 1.2

#### Build instructions

```bash
$ git clone https://github.com/udacity/SFND_Unscented_Kalman_Filter.git unscented-kalman-filter-project
$ mkdir -p unscented-kalman-filter-project/build && cd unscented-kalman-filter-project/build
$ cmake .. && make
$ ./ukf_highway
```

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment
and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.

