# Lesson 1-3 Clustering Obstacles

Segmenting points allows us to recognize the obstacles. It would be great to break up and group those obstacle points, especially for the multiple object tracking with cars, pedestrians, and bicyclists, etc. One way to do that grouping and cluster point cloud data is called euclidean clustering.

### I. Euclidean Clustering

This method associates groups of points by how close together they are. To do a nearest neighbor search efficiently, a **KD-Tree** data structure which could speed up the look up time from *O(n)* to *O(log(n))*. By grouping points into regions in a KD-Tree, you can avoid calculating distance for possibly thousands of points just because you know they are not even considered in a close enough region.