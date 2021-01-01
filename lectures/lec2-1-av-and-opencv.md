# Lesson 2-1 Autonomous Driving and OpenCV

This lesson will give an introduction about the autonomous driving and computer vision.



### I. Levels of Autonomous Driving

For years, there are many vehicles on the road equipped with *Advanced Driver Assistance Systems (ADAS)*, which assist the driver in various driving tasks, such as *Forward Collision Warning (FCW)*, *Adaptive Cruise Control (ACC)*, etc. But they only provide some degree of automation in a selected number of driving situations, e.g. highways, etc. The driver still must remain engaged in during the driving and monitor the environment.

For an automation Level 3 and above, the drivers are no longer required to monitor the environment, even though they must be available to take back the control at all times. Therefore, it means the responsibility for the driving tasks is with the car company/manufacturer from a legal viewpoint.

The graph below from the NHTSA website shows the *Levels of autonomous driving* defined by the SAE.

<img src="media/nhtsa-sae-automation-levels.png" width="800" height="400" />



### II. Autonomous Vehicle Sensor Sets

The variety of the sensor sets for autonomous vehicles depends on the autonomy level, on the area of application, and engineering team's design.

- Uber ATG AVs
    * 7 cameras, 1 Lidar, inertial measurement units (IMUs), Radars, and storage/compute units

- Mercedes Benz AVs
    * cameras, Lidar and Radars, similar to Uber vehicles
    * feature a setup of synchronized cameras, which is able to measure depth information

- Tesla Autopilot
    * cameras with partially overlapping fields of view, forward-facing Radar (160m), and 360-degree ultrasonic sonar (8m)
    * no Lidar

#### Camera

- Limitations
    * poor performance at limited-vision scenarios, e.g. night, snow, heavy rain, fog, etc.

#### Radar

- Advantages
    * gives not only **distance** to the obstacle (time of flight), but also the **relative speed** (shifted frequency)
    * robust against adverse weather conditions, e.g. thick fog, heavy snow, etc.
    * best at identifying *large* objects with *good reflective properties*

- Limitations
    * poor performance on detecting *small* and *"soft"* objects, like humans and animals

#### Lidar

- Limitations
    * poor performance at adverse environment conditions, similar to the camera

#### Sensor Selection Criteria

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
