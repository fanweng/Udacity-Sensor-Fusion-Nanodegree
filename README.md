# Udacity Sensor Fusion Nanodegree Program

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, which is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intensity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very practical sensor for doing things like cruise control where its important to know how fast the car in front of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing Lidar's high resolution imaging with radar's ability to measure velocity of objects we can get a better understanding of the surrounding environment than we could using one of the sensors alone.

### Project 1 - Lidar Obstacle Detection

In this project, I processed multiple point clouds data files from Lidar sensor, and detected the cars or other obstacles on a city street. The detection pipeline was implemented by the **Voxel Grid and ROI based filtering**, **3D RANSAC segmentation**, **Euclidean clustering based on KD-Tree**, and **bounding boxes**.

My final result is shown below, where the green points represent the street surface and the obstacles are marked in the red boxes.

<img src="Lidar_Obstacle_Detection/media/obstacle-detection-fps-final.gif" width="800" height="400" />

Please check the detailed project description in the [Lidar_Obstacle_Detection/README.md](Lidar_Obstacle_Detection/README.md).

### Project 2 - 3D Object Tracking

This project tracks the preceding vehicle in the same lane and estimates the time-to-collision (TTC) based on both camera images and Lidar data. To build up the camera TTC estimation pipeline, I implemented the keypoint detection, descriptor extraction, and methods matched keypoints between successive images. With the help of 3D bounding boxes, I was able to extract keypoints corresponding to the preceding vehicle and calculate the TTC based on relative distance between matched keypoints in two successive images. Matched keypoints also contributed to match 3D bounding boxes in the Lidar point cloud, so that Lidar pipeline could estimate the TTC using the closest distances of the bounding boxes to the ego vehicle in two successive frame.

The output clip is shown below, where the preceding vehicle is tracked with a green box, 3D Lidar points on the vehicle trunk are projected to the 2D frame (green points). TTC estimations based on Lidar and camera are reported on the top.

<img src="Camera/Lesson-7-Project-3D-Object-Tracking/results/ttc-estimations.gif" width="900" height="300" />

Please check the detailed project description in the [Camera/Lesson-7-Project-3D-Object-Tracking/README.md](Camera/Lesson-7-Project-3D-Object-Tracking/README.md).