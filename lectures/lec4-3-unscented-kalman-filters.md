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

## II. Unscented Kalman Filters (UKF)

The EKF would give a poor performance when the predict function `f(x)` and update function `h(x)` are highly nonlinear. The unscented Kalman filter (UKF) uses a deterministic sampling technique known as the *unscented transformation (UT)* to pick a minimal set of sample points (called **sigma points**) around the mean.

<img src="media/ukf-roadmap.png" width="700" height="350" />

#### Sigma points

The number of sigma points depends on the dimension of state vector. For a simpler case, we can use a 2D state vector with `px` and `py` which gives 5 sigma points. 

<img src="media/num-sigma-points.png" width="700" height="500" />

The following equations calculate the 5 sigma points for a 2D state vector.

<img src="media/generate-sigma-points.png" width="700" height="500" />

The source code: [ukf.cpp](../Kalman_Filters/sigma-points-exercise/ukf.cpp)

#### UKF augmentation


<img src="media/ukf-augmentation.png" width="700" height="700" />

## V. Equation Cheatsheet

[Sensor Fusion EKF Reference.pdf](../Kalman_Filters/sensor-fusion-ekf-reference.pdf)

[Quick Reference to the Eigen Library](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)