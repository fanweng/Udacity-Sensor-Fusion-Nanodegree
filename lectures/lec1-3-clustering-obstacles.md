# Lesson 1-3 Clustering Obstacles

Segmenting points allows us to recognize the obstacles. It would be great to break up and group those obstacle points, especially for the multiple object tracking with cars, pedestrians, and bicyclists, etc. One way to do that grouping and cluster point cloud data is called euclidean clustering.

### I. Euclidean Clustering

This method associates groups of points by how close together they are. To do a nearest neighbor search efficiently, a **KD-Tree** data structure which could speed up the look up time from *O(n)* to *O(log(n))*. By grouping points into regions in a KD-Tree, you can avoid calculating distance for possibly thousands of points just because you know they are not even considered in a close enough region.

### II. Implement KD-Tree Quiz

A KD-Tree is a binary tree that splits points between alternating axes. By separating space by splitting regions, nearest neighbor search can be made much faster when using an algorithm like euclidean clustering.

The node of KD-Tree looks like:
```c++
struct Node
{
    std::vector<float> point;   // point coordinates
    int id;                     // unique index to tell which point is referenced from the point cloud
    Node* left;
    Node* right
    ... ...
}
```

#### Insert 2D points to KD-Tree

At each depth of the KD-Tree, one coordinate value will be compared, either `x` or `y`. It is also called splitting the `x` or `y` region, which is defined by `depth % 2`: an even `depth` value will split `x` region, an odd `depth` value will split `y` region. The inserting point goes to the **left child** if the coordinate value is **smaller** than current node, goes to the **right child** if it is **larger**. ([dd0bf1f](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/dd0bf1fb72fdd0b7486e4c3ed39adc72c0ec7834))

<img src="media/kdtree-insert-2d-points-quiz.png" width="800" height="400" />

#### Search points in a KD-Tree

Points within a distance of distanceTol are considered to be nearby. The KD-Tree is able to split regions and allows certain regions to be completely ruled out, speeding up the process of finding nearby neighbors.

The naive approach of finding nearby neighbors is to go through every single point in the tree and compare their distances with the target, selecting point indices that fall within the distance tolerance of the target. Instead with the KD-Tree you can compare distance within a boxed square that is 2 x distanceTol for length, centered around the target point. If the current node point is within this box then you can directly calculate the distance and see if the point id should be added to the list of nearby ids. Then you see if your box crosses over the node division region and if it does compare that next node. You do this recursively, with the advantage being that if the box region is not inside some division region you completely skip that branch.