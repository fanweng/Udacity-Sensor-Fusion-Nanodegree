# Lecture 3-4 Clustering and Tracking

### I. Clustering

A radar can generate detections from plentitude of scattering points on the target.

- Clustering algorithm
    * groups the detection points based on their **proximity** measured by the **euclidean distance** between those points
    * all the detection points that are **within size of the target** are considered as one cluster, merged into a **centroid position**
    * each cluster is assigned a **new range and velocity**, which is the mean of measured range and velocity of the points within cluster

### II. Kalman Tracking

The purpose of Kalman filter is to estimate the **state** of a tracked vehicle, e.g. *position*, *velocity*, *acceleration*, or other properties. It uses measurements with noise and variation and other inaccuracies, produces values that tend to be closer to the true values. It's the vital algorithm to the majority of all modern radar tracking systems.

<img src="media/kalman-filter.png" width="900" height="600" />

- Kalman filter procedure:
    * Prediction
        + use vehicle's motion model, e.g. `Xnew = Xprev + v * t`
        + predict the next state of vehicle using the current state (position) and velocity from the previous timestamp
    * Update
        + use noisy measurement data from sensors
        + combine the data with prediction data to produce a best-possible estimate of the state

`trackingKF` MATLAB object tutorial [HERE](https://www.mathworks.com/help/driving/ref/trackingkf.html).

Explanation of Kalman filter with MATLAB videos [HERE](https://www.youtube.com/watch?v=mwn8xhgNpFY&list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr).

### III. MATLAB Sensor Fusion

In the [sensor_fusion_with_radar.m](../Radar/exercises/sensor_fusion_with_radar.m), Kalman filtering is implemented in the simulated environment using MATLAB.