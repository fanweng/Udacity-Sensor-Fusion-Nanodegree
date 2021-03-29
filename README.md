# Udacity Sensor Fusion Nanodegree Program

In this program, I have learned knowledge in two different sensors, Lidar and Radar. Code has been developed to detect obstacles using Lidar point cloud data, to track the object using Camera images, to detect range and velocity of targe based on Radar data, and to fuse Lidar/Radar measurement to predict the object's movement using Kalman Filter.

|  Criteria  |  Lidar  |  Radar  |  Camera  |
|------------|---------|---------|----------|
| **Range**      | Meters to 200m | Meters to 200m | Only stereo camera setup can measure distance up to 80m |
| **Spatial Resolution** | High, 0.1 degree due to short wavelength laser | Cannot resolve small features | Defined by optics, pixel size of image and its signal-to-noise ratio |
| **Robustness in Darkness** | Excellent, due to active | Excellent, due to active | Reduced |
| **Robustness in Rain, Snow, Fog** | Limited, due to optical | Best | Limited, due to optical |
| **Classification of Objects** | Some level of classification by 3D point clouds | Not too much classification | Excellent at classification |
| **Perceiving 2D Structures** | N/A | N/A | The only sensor that is able to interpret traffic signs, lane markings, traffic lights |
| **Measure Speed** | Approximate speed by using successive distance measurement | Measure velocity by exploiting the Doppler frequency shift | Can only measure time to collision by observing the displacement of objects on the image plane |
| **System Cost** | More expensive | Compact and affordable | Compact and affordable |
| **Package Size** | Hard to integrate | Easily integrated | Easily integrated for mono cameras, but stereo camera setup is bulky |
| **Computational Requirements** | Little | Little | Significant |

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

### Lecture Notes

[Lecture 1-1 Lidar and Point Cloud](lectures/lec1-1-lidar-and-point-clouds.md)

[Lecture 1-2 Point Cloud Segmentation](lectures/lec1-2-point-cloud-segmentation.md)

[Lecture 1-3 Clustering Obstacles](lectures/lec1-3-clustering-obstacles.md)

[Lecture 1-4 Work with PCD](lectures/lec1-4-work-with-pcd.md)

[Lecture 2-1 AV and OpenCV](lectures/lec2-1-av-and-opencv.md)

[Lecture 2-2 Collision Detection System](lectures/lec2-2-collision-detection-system.md)

[Lecture 2-3 Track Image Features](lectures/lec2-3-track-image-features.md)

[Lecture 2-4 Combine Camera Image and Lidar Data](lectures/lec2-4-combine-camera-lidar.md)

[Lecture 3-1 Radar Principles](lectures/lec3-1-radar-principles.md)

[Lecture 3-2 Range and Doppler Estimation](lectures/lec3-2-range-doppler-estimation.md)

[Lecture 3-3 Clutter and CFAR](lectures/lec3-3-clutter.md)

[Lecture 3-4 Clustering and Tracking](lectures/lec3-4-clustering-and-tracking.md)

[Lecture 4-1 Kalman Filters](lectures/lec4-1-kalman-filters.md)

[Lecture 4-2 Lidar/Radar Fusion with Kalman Filters](lectures/lec4-2-lidar-radar-fusion-with-KF.md)

[Lecture 4-3 Unscented Kalman Filters](lectures/lec4-3-unscented-kalman-filters.md)