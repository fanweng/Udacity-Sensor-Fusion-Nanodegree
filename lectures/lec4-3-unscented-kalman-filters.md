# Lecture 4-3: Unscented Kalman Filters

## I. CTRV Model

In the extended Kalman filter (EKF) lesson, the *constant velocity (CV)* model was used. But it has limitation when a vehicle drives straight at first and then goes into a turn.

There are many other motion models:
- constant turn rate and velocity magnitude model (CTRV)
- constant turn rate and acceleration (CTRA)
- constant steering angle and velocity (CSAV)
- constant curvature and acceleration (CCA)

#### CTRV state vector

<img src="media/ctrv-model.png" width="750" height="300" />

#### CTRV equations

<img src="media/ctrv-equations.png" width="900" height="600" />

