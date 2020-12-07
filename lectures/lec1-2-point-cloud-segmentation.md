# Lesson 1-2 Point Cloud Segmentation

### I. Segmentation

Segment out a group of points belong to some certain object in the environmental scene from the point cloud.

### II. RANSAC

RANSAC stands for Random Sample Consensus, a method for detecting outliers in data.

- RANSAC runs iterations and returns the model with the best fit
    * each iteration randomly picks a subsample of the data and fits a model through it, such as line or a plane
    * the iteration with the highest number of inliers or lowest noise is used as the best model

- RANSAC version to implement in the quiz uses the **smallest possible subset of points** to fit
    * line (two points), plane (three points)
        + iterate through every remaining points and calculate its distance to the model
        + points that are within a certain distance to the model are counted as **inliers**
        + the iteration has the **highest number of inliers** is the best model

- Other RANSAC version samples **some percentage of the model points** to fit
    * for example, 20% of the total points, then fit a line to that
        + the error of that line is calculated
        + the iteration with the **lowest error** is the best model

#### RANSAC 2D Quiz

`Lidar_Obstacle_Detection/src/quiz/ransac/ransac2d.cpp` implements RANSAC for fitting a line in 2D point cloud data with outliers. `Ransac1()` is my solution and `Ransac2()` is the solution provided by Udacity. The basic idea is:

1. Randomly pick out two points from the point cloud data, fit a line with the two points.
2. Iterate the rest of points in the point cloud data, calculate its distance to the fitted line. If distance is within the tolerance, we consider it as an inlier.
3. Repeat the same procedure, find a group of points with most number of inliers.

<img src="media/line-fitting-formula.JPG" width="800" height="100" />