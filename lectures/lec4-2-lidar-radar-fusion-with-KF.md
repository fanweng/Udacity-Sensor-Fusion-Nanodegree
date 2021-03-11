# Lidar and Radar Fusion with Kalman Filter

## I. Fusion Algorithm

- **First measurement**
    * receive initial measurement of the obstacle's position relative to the ego vehicle
- **Initialization of state and covariance metrics**
    * initialize the obstacle's position based on the first measurement
- Ego vehicle receives another measurement after a `dt`
- **Predict**
    * estimate the obstacle's position after the `dt` using constant velocity model
- **Update**
    * predicted location and measured location are combined to give an updated location. Kalman filter will put weight on them depending on the uncertainty of each value
- Loop over another measurement after `dt` and predict/update the location

<img src="media/lidar-radar-fusion-kf-algo.png" width="700" height="400" />

## II. Update/Predict Processes

Measurement updates for Lidar and Radar are usually asynchronous. If they arrive simultaneously, use either one of the sensors to update/predict, and then update with the other sensor measurement.

`x`: mean state vector, containing object's position and velocity

`P`: state covariance matrix, which contains the uncertainty of the object's position and velocity

<img src="media/kf-update-predict-flow.png" width="900" height="250" />

To implement the update and predict processes in C++, we need the [Eigen Library](http://eigen.tuxfamily.org/) which the version used in this course can be downloaded [HERE](https://d17h27t6h515a5.cloudfront.net/topher/2017/March/58b7604e_eigen/eigen.zip).

The source code: [kf_filter_equations.cpp](../Kalman_Filters/kf_filter_equations.cpp).

<img src="media/kf-equations.png" width="600" height="400" />


## IV. Equation Cheatsheet

[Sensor Fusion EKF Reference.pdf](../Kalman_Filters/sensor-fusion-ekf-reference.pdf)