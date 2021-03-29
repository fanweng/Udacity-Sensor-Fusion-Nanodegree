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

### Project 2 - 3D Object Tracking based on Camera

This project tracks the preceding vehicle in the same lane and estimates the time-to-collision (TTC) based on both camera images and Lidar data. To build up the camera TTC estimation pipeline, I implemented the keypoint detection, descriptor extraction, and methods matched keypoints between successive images. With the help of 3D bounding boxes, I was able to extract keypoints corresponding to the preceding vehicle and calculate the TTC based on relative distance between matched keypoints in two successive images. Matched keypoints also contributed to match 3D bounding boxes in the Lidar point cloud, so that Lidar pipeline could estimate the TTC using the closest distances of the bounding boxes to the ego vehicle in two successive frame.

The output clip is shown below, where the preceding vehicle is tracked with a green box, 3D Lidar points on the vehicle trunk are projected to the 2D frame (green points). TTC estimations based on Lidar and camera are reported on the top.

<img src="Camera/Lesson-7-Project-3D-Object-Tracking/results/ttc-estimations.gif" width="900" height="300" />

Please check the detailed project description in the [Camera/Lesson-7-Project-3D-Object-Tracking/README.md](Camera/Lesson-7-Project-3D-Object-Tracking/README.md).

### Project 3 - Velocity and Range Detection based on Radar

This project first defines a target with certain velocity and position, as well as Radar specifications. It then propagates the Radar wave signal based on the Frequency Modulated Continuous Wave (FWCW) model. Range and Doppler 2D FFT is applied to the received signal to determine the range and velocity of the target. At the end, 2D Constant False Alarm Rate (CFAR) detector is performed on the 2D FFT, which detects the target.

The result looks like that Matlab plot below, indicating a target is detected with a range of about 81m and a velocity of about -20m/s.

<img src="Radar/project/media/2d-cfar-rdm.png" width="900" height="500" />

Please check the detailed project description in the [Radar/project/README.md](Radar/project/README.md).

### Project 4 - Kalman Filters for Sensor Fusion

In the final project, each moving car except the ego vehicle has been assigned an unscented Kalman Filter (UKF). Lidar and Radar data for the moving cars are continuously fed into their UKF respectively. Those data is processed within the UKF so that the position and velocity of each car can be estimated through predict-update cycles.

The result below illustrates a highway scenario. The ego car is green while the other traffic cars are blue. The red spheres above cars represent the lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The green spheres above cars represent the UKF predicted path that cars would move in the near future. 

<img src="Kalman_Filters/unscented-kalman-filter-project/media/ukf_output.gif" width="900" height="400" />

Please check the detailed project description in the [Kalman_Filters/unscented-kalman-filter-project/README.md](Kalman_Filters/unscented-kalman-filter-project/README.md).