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

## IV. Equation Cheatsheet

[Sensor Fusion EKF Reference.pdf](../Kalman_Filters/sensor-fusion-ekf-reference.pdf)