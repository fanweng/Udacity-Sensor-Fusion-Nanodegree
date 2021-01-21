# Lesson 2-4 Combining Camera and Lidar

Single-sensor approach is limited by the fact that each sensor has its own weakness in some situation. This lesson will show how to combine 2D camera image data and 3D Lidar data to improve the tracking process results.



### I. Project Lidar Points to Camera Images


#### Exercise: Show Lidar point in a top view

Remove the Lidar points representing the road surface by eliminating the points below a certain height `roadSurfaceLvl` in z-axis. Draw the rest of Lidar points in a gradient color from red (x = 0.0m) to green (x = 20.0m). ([9b25ab0](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/9b25ab06b20bf1c51b3580576098f9bc01265a75))

<img src="media/show-lidar-top-view.png" width="800" height="350" />