# Lesson 1-4 Working with Real PCD

This lesson will apply the segmentation and clustering techniques to real point cloud data from a self-driving car.

- Real-world challenges for Lidar
    * Environmental conditions: heavy rains, stand storms, and anything reflects/scatters laser
    * Ghost objects caused by highly reflective surfaces: water sprays from other cars, etc

- Downsampling
    * Sending out the entire point cloud data is too heavy for the vehicle internal network
    * Use **stixels**
        + Stixels are segments which represent sensor data in a compact fashion while retaining the underlying semantic and geometric properties
        + An example of stixels ([Ref](https://www.mathworks.com/matlabcentral/fileexchange/65347-stixel-world)). Use *height*, *width*, and *number* of rectangles to represent objects, instead of individual points.

<img src="media/stixels-example.png" width="800" height="400" />    