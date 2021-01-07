# Lesson 2-2 Collision Detection System

This lesson will introduce the basics of the collision detection model, will show how to estimate the time-to-collision (TTC) using Lidar or camera.



### I. Collision Detection

A Collision Avoidance System (CAS) is an active safety feature that either warns the driver or apply the brakes autonomously in the event of am imminent collision with an object in the path of driving. CAS continuously estimates the TTC and acts when the TTC falls below a threshold.

- Two assumption models to compute the TTC
    * **Constant Velocity Model (CVM):** relative velocity between two objects is constant
        + need to measure *distance* to the preceding object with a constant *time interval* between measurements
        + much simpler
    * **Constant Acceleration Model (CAM):** relative acceleration between two objects is constant
        + complex but more accurate

<img src="media/ttc-estimation-models.png" width="600" height="300" />