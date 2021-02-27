# Lecture 3-4 Clustering and Tracking

### I. Clustering

A radar can generate detections from plentitude of scattering points on the target.

- Clustering algorithm
    * groups the detection points based on their **proximity** measured by the **euclidean distance** between those points
    * all the detection points that are **within size of the target** are considered as one cluster, merged into a **centroid position**
    * each cluster is assigned a **new range and velocity**, which is the mean of measured range and velocity of the points within cluster

### II. Kalman Tracking

