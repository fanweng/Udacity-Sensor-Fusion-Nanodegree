# Unscented Kalman Filter Project

In this project, Unscented Kalman Filter (UKF) is implemented to estimate the state of multiple cars on a highway using noisy lidar and radar measurements.

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The traffic cars will be accelerating and altering their steering to change lanes. The `Z` axis is not taken into account for tracking, so you are only tracking along the `X/Y` axis.

Each of the traffic car's has its own UKF object generated for it, and will update each individual one during every time step using Constant Turn Rate and Velocity (CTRV) motion model.

The accuracy will be evaluated by the Root Mean Squared Error (RMSE) over each time step and for each car.

## I. System Preparations

#### Starter code

[https://github.com/udacity/SFND_Unscented_Kalman_Filter](https://github.com/udacity/SFND_Unscented_Kalman_Filter)

#### Build requirements

- cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
- make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
- gcc/g++ >= 5.4
    * Linux: gcc/g++ is installed by default on most Linux distros
- PCL 1.2

#### Build instructions

```bash
$ git clone https://github.com/udacity/SFND_Unscented_Kalman_Filter.git unscented-kalman-filter-project
$ mkdir -p unscented-kalman-filter-project/build && cd unscented-kalman-filter-project/build
$ cmake .. && make
$ ./ukf_highway
```

## II. Debugging Details

In the `highway.h`, there are a number of parameters we can modify for debugging purpose.

- `trackCars` list can toggle on/off cars for UKF object to track
- `projectedTime` and `projectedSteps` controls the visualization of predicted position in the future
- `visualize_pcd` sets the visualization of Lidar point cloud data

```c++
// Set which cars to track with UKF
std::vector<bool> trackCars = {true,true,true};
// Visualize sensor measurements
bool visualize_lidar = true;
bool visualize_radar = true;
bool visualize_pcd = false;
// Predict path in the future using UKF
double projectedTime = 0;
int projectedSteps = 0;
```

## III. Code Walkthrough

1. Initialize the UKF attributes ([e4adb2a](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/e4adb2ac2c7c16424c6cbbb7a57fc9c9f5e9de8e#diff-03a16d9c10d902a123ceaa597573c293db78ddd2994a7a480ef9ea7b6a1c7f82))

- dimension of the state vector `n_x_`
- state vector `x_`
- covariance matrix `P_`
- dimension of the augmented state vector `n_aug_`
- predicted sigma points matrix `Xsig_pred_`
- sigma points weights vector `weights_`
- standard deviation of longitudinal acceleration noise `std_a_`
- standard deviation of yaw acceleration noise `std_yawdd_`
- sigma points spreading parameter `lambda_`

2. Implement `UKF::ProcessMeasurement()` ([7f38296](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/7f38396cc44da4bca32236a2148a6df328a08e06#diff-03a16d9c10d902a123ceaa597573c293db78ddd2994a7a480ef9ea7b6a1c7f82))

For the very first incoming measurement, state vector `x_`, covariance matrix `P_`, and timestamp `time_us_` are initialized according to the raw data `meas_package.raw_measurements_` and `meas_package.timestamp_`.

For the following measurements, timestamp `time_us_` is recorded, a sequence of functions are called to `Prediction()` and `UpdateLidar()`/`UpdateRadar()`.

3. Implement `UKF::Prediction()` ([6f447da](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/6f447dac02bdd7c392958fd15454d627c6688d0e#diff-03a16d9c10d902a123ceaa597573c293db78ddd2994a7a480ef9ea7b6a1c7f82))

The prediction process is the same for both Lidar and Radar measurements.

- creates an augmented mean vector `x_aug` and augmented state covariance matrix `P_aug`
- generate sigma points matrix `Xsig_aug` for previously estimated state vector
- predict sigma points matrix `Xsig_pred_` for the current state vector 
- predict the state mean `x_` and covariance `P_` using weights and predicted sigma points

4. Implement `UKF::UpdateLidar()` and `UKF::UpdateRadar()` ([99245dd](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/99245dd48e4bad2459ba9afe14fe887e730a3829#diff-03a16d9c10d902a123ceaa597573c293db78ddd2994a7a480ef9ea7b6a1c7f82), [e67e5c3](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/e67e5c3f2864792eba6d63b731b09378d15bb142#diff-03a16d9c10d902a123ceaa597573c293db78ddd2994a7a480ef9ea7b6a1c7f82))

The steps to update Lidar and Radar measurements are similar, except Lidar points are in the **Cartesian** coordinates but Radar points are in the **Polar** coordinates. Therefore, they differ in the measurement dimension `n_z`, dimension of matrices, and the transformation equations.

Generally, they follow the same steps to update the measurement.

- transform the predicted sigma points `Xsig_pred_` into measurement space `Zsig` based on the sensor types
- calculate the mean state `z_` and covariance matrix `S` with noise considered
- calculate cross-correlation matrix `Tc` between state space and measurement space
- calculate the Kalman gain `K`
- update the state vector `x_` and covariance `P_`

5. Test run

The screenshot shown below is one of the simulation moments. The ego car is green while the other traffic cars are blue. The red spheres above cars represent the `(x,y)` lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The green spheres above cars represent the predicted path that cars would move in the near future.

On the left-hand side, the root mean squared errors (RMSE) for position `(x,y)` and velocity `(Vx, Vy)` are calculated in realtime, which represent the prediction accuracy.

<img src="media/ukf_result.png" width="800" height="500" />

## IV. Results

I experimented different initial values for the state vector `x_`, covariance matrix `P_`, standard deviation of longitudinal acceleration noise `std_a_`, standard deviation of yaw acceleration noise `std_yawdd_`. And the following parameters serve the best result. The RMSE values are always within the thresholds during the simulation.

```c++
std_a_ = 2.0;
std_yawdd_ = 2.0

/* For Lidar */
x_ << meas_package.raw_measurements_[0],
    meas_package.raw_measurements_[1],
    0,
    0,
    0;

P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
    0, std_laspy_ * std_laspy_, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;

/* For Radar */
x_ << rho * cos(phi),
    rho * sin(phi),
    0,
    0,
    0;

P_ << std_radr_*std_radr_, 0, 0, 0, 0,
    0, std_radr_ * std_radr_, 0, 0, 0,
    0, 0, std_radrd_ * std_radrd_, 0, 0,
    0, 0, 0, std_radphi_ * std_radphi_, 0,
    0, 0, 0, 0, 1;
```

<img src="media/ukf_output.gif" width="800" height="500" />
