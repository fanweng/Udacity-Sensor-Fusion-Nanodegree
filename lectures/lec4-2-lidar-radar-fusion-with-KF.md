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

#### State prediction

The following diagram shows the detailed steps in the predict process.

<img src="media/detailed-predict-process.png" width="600" height="200" />

The **state prediction** uses the kinematic equation below. The *motion noise* and *process noise* refer to the uncertainty of position when predicting location. The *measurement noise* refers to the uncertainty in sensor measurement.

<img src="media/state-transition-matrix.png" width="500" height="250" />

#### Process covariance matrix Q

The **state covariance matrix update equation** and **Q** is:

<img src="media/process-covariance-matrix-equation.png" width="400" height="200" />

To deduce the Q value shown above, we need to model acceleration as a random noise.

## IV. Equation Cheatsheet

[Sensor Fusion EKF Reference.pdf](../Kalman_Filters/sensor-fusion-ekf-reference.pdf)