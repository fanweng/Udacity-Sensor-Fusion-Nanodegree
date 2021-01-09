# Lesson 2-2 Collision Detection System

This lesson will introduce the basics of the collision detection model, will show how to estimate the time-to-collision (TTC) using Lidar or camera.



### I. Collision Detection

A Collision Avoidance System (CAS) is an active safety feature that either warns the driver or apply the brakes autonomously in the event of am imminent collision with an object in the path of driving. CAS continuously estimates the TTC and acts when the TTC falls below a threshold.

- Two assumption models to compute the TTC
    * **Constant Velocity Model (CVM):** relative velocity between two objects is constant
        + need to measure *distance* to the preceding object with a constant *time interval* between measurements
        + much simpler, and the following course will use this
    * **Constant Acceleration Model (CAM):** relative acceleration between two objects is constant
        + complex but more accurate

<img src="media/ttc-estimation-models.png" width="600" height="300" />



### II. Estimate with Lidar

Lidar measures the distance to the closest points on the preceding vehicle. Based on the CVM, we have the following equations to compute the *relative velocity* and the *TTC*.

<img src="media/ttc-estimation-lidar.png" width="850" height="200" />

In order to derive a stable TTC measurement from the given point cloud, we need to 1) remove points of the road surface and 2) remove measurements with low reflectivity. The following image shows the point cloud after preprocessing.

<img src="media/ttc-estimation-lidar-preprocess.png" width="800" height="300" />

The code example introduces a data structure `LidarPoint` as below. We need to find the distance to the closest Lidar point in the path of driving. In the figure below, the tailgate of the preceding vehicle is measured at *t0* (green) and *t1* (red). Even though Lidar is a reliable sensor, erroneous measurement will still occur, e.g. the individual points outside the tailgate measured. We will perform a more robust calculation of minimum distance dealing with those outliers and also look at TTC estimation from camera.

```c++
struct LidarPoint {
    double x, y, z; // point position in meter, x (forward), y (left), z (up)
    double r;       // point reflectivity ranging between [0, 1]
};
```

<img src="media/ttc-estimation-lidar-points.png" width="800" height="300" />

In the [compute_ttc_lidar.cpp](../Camera/Lesson-3-Engineering-a-Collision-Detection-System/Estimating-TTC-with-Lidar/TTC_lidar/src/compute_ttc_lidar.cpp), filter out the points outside the ego-lane, iterate point clouds for *t0* and *t1*, find out the closest points respectively and compute the TTC. ([acd6ff3](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/acd6ff380dad753208ca899a5ca389aba2814a0f))



### III. Estimate with Camera

Camera is a passive sensor which is not possible to measure the travel time of light. Therefore, mono camera is not able to measure metric distance. Stereo camera, however, can measure the distance based on two images captured at the same time and the camera geometric.

Despite the limitations of mono camera, there is still a way to compute the TTC WITHOUT knowing the distance. Based on CVM, the TTC is estimated with the scale change of the vehicle projection on the image plane. In the illustration below, *H* is the height of vehicle, *h0* and *h1* are relative heights of vehicle on the image plane, *f* is the focal length.

Thus, we need to know the ratio of heights *h1/h0* to compute the TTC.

<img src="media/ttc-estimation-camera.png" width="800" height="600" />

- How to get the geometry of the heights?
    * **Bounding box:** use its width or height
        + neural network is trained to locate the vehicle with bounding box
        + but bounding box doesn't always reflect the true vehicle dimension, leading to estimation errors
    * **Texture keypoints:** use the distance between them
        + locate uniquely identifiable keypoints on the vehicle, track them from one frame to the next
        + the height ratio can be replaced by the mean or median of all distance ratios *dk/dk'*

<img src="media/ttc-estimation-camera-keypoints.png" width="600" height="350" />

In the [compute_ttc_camera.cpp](../Camera/Lesson-3-Engineering-a-Collision-Detection-System/Estimating-TTC-with-Camera/TTC_camera/src/compute_ttc_camera.cpp) of `computeTTCCamera`, use the median of the calculated ratios to replace the average value, which is more robust. ([d1a655f](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/d1a655f66e23f061c8fe89e841d08694170355aa))



### IV.Design of TTC Computation

- Camera building blocks
    * Main idea
        + compute keypoint matches between successive images, and observe the relative distances between them over time
    * Problem
        + need to isolate keypoints on the preceding vehicle, excluding the road surface, static objects, etc.

- Lidar building blocks
    * Main idea
        + compute distance to preceding vehicle from 3D point cloud at successive moments in time
    * Problem
        + need to isolate 3D points on the preceding vehicle, excluding the road surface, static objects, etc.
        + cropping isn't reliable enough, especially when the preceding vehicle isn't directly in front of the sensor

- Sensor fusion building blocks
    * Main idea
        + use camera-based object classification to cluster Lidar points, and compute TTC from 3D bounding boxes

<img src="media/ttc-computation-design.png" width="500" height="600" />