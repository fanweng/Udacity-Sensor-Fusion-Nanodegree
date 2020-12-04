# Lidar Obstacle Detection Project 

[//]: # (<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />)

### I. Preparation on Ubuntu

For this project, the virtual Ubuntu environment was provided by the Udacity. Install the Point Cloud Library (PCL) and clone the project starter code [SFND_Lidar_Obstacle_Detection](https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git) to the Ubuntu environment.

```bash
$ sudo apt install libpcl-dev
$ git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git ./Lidar_Obstacle_Detection
```

The directory structure of the starter code looks like:
- src
    * render
        + `box.h`: struct definition for box objects
        + `render.h`, `render.cpp`: classes and methods for rendering objects
    * sensors
        + `lidar.h`: functions using ray casting for creating PCD
    * `environment.cpp`: main file for using PCL viewer and processing/visualizing PCD
    * `processPointClouds.h`, `processPointClouds.cpp`: functions for filtering, segmenting, clustering, boxing, loading and saving PCD

Try compiling the lidar simulator and run it. A window with a simulated highway environment should pop up.
```bash
$ cd Lidar_Obstacle_Detection
$ mkdir build && cd build
$ cmake ..
$ make
$ ./environment
```

### Exercises

- Create a lidar object and render the ray