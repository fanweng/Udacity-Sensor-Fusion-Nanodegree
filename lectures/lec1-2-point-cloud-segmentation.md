# Lesson 1-2 Point Cloud Segmentation

### I. Segmentation

Segment out a group of points belong to some certain object in the environmental scene from the point cloud.

### II. RANSAC

RANSAC stands for Random Sample Consensus, a method for detecting outliers in data.

- RANSAC runs iterations and returns the model with the best fit
    * each iteration randomly picks a subsample of the data and fits a model through it, such as line or a plane
    * the iteration with the highest number of inliers or lowest noise is used as the best model

<img src="media/ransac-line-model.png" width="800" height="400" />

- RANSAC version to implement in the quiz uses the **smallest possible subset of points** to fit
    * line (two points), plane (three points)
        + iterate through every remaining points and calculate its distance to the model
        + points that are within a certain distance to the model are counted as **inliers**
        + the iteration has the **highest number of inliers** is the best model

- Other RANSAC version samples **some percentage of the model points** to fit
    * for example, 20% of the total points, then fit a line to that
        + the error of that line is calculated
        + the iteration with the **lowest error** is the best model