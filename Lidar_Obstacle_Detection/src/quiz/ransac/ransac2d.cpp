/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }

    // Add outliers
    int numOutliers = 10;
    while (numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}


// My Solution of RANSAC for a Line
std::unordered_set<int> RansacLine1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    while (maxIterations-- > 0) {
        // Randomly sample subset
        pcl::PointXYZ point1 = cloud->points.at(rand() % (cloud->points.size()));
        pcl::PointXYZ point2 = cloud->points.at(rand() % (cloud->points.size()));
        // Fit line, Ax+By+C=0, z is always zero for 2D line
        float A, B, C;
        A = point1.y - point2.y; // y1 - y2
        B = point2.x - point1.x; // x2 - x1
        C = point1.x*point2.y - point2.x*point1.y; // x1*y2 - x2*y1

        // Measure distance between every point and fitted line
        std::unordered_set<int> inliersTemp;
        for (auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            float d = fabs(A*((*it).x)+B*((*it).y)+C)/sqrt(A*A+B*B); // |A*x3+B*y3+C|/(A^2+B^2)
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceTol) {
                inliersTemp.insert(it - cloud->begin());
            }
        }
        if (inliersTemp.size() > inliersResult.size()) {
            inliersResult = inliersTemp;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}


// Official Solution of RANSAC for a Line
std::unordered_set<int> RansacLine2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    while (maxIterations--) {
        std::unordered_set<int> inliers;
        while (inliers.size() < 2)
            inliers.insert(rand()%(cloud->points.size()));

        float x1, y1, x2, y2;
        auto it = inliers.begin();
        x1 = cloud->points[*it].x;
        y1 = cloud->points[*it].y;
        it++;
        x2 = cloud->points[*it].x;
        y2 = cloud->points[*it].y;

        float A, B, C;
        A = y1 - y2;
        B = x2 - x1;
        C = x1*y2 - x2*y1;

        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip the two points that are sample points
            if (inliers.count(index) > 0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;

            float d = fabs(A*x3+B*y3+C)/sqrt(A*A+B*B);
            if (d <= distanceTol) {
                inliers.insert(index);
            }
        }
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << "ms" << std::endl;
    return inliersResult;
}


// My Solution of RANSAC for a Plane
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    while (maxIterations-- > 0) {
        // Randomly sample subset
        pcl::PointXYZ point1 = cloud->points.at(rand() % (cloud->points.size()));
        pcl::PointXYZ point2 = cloud->points.at(rand() % (cloud->points.size()));
        pcl::PointXYZ point3 = cloud->points.at(rand() % (cloud->points.size()));
        // Fit a plane, Ax+By+Cz+D=0
        float A, B, C, D;
        A = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y); // (y2 - y1)(z3 - z1) - (z2 - z1)(y3 - y1)
        B = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z); // (z2 - z1)(x3 - x1) - (x2 - x1)(z3 - z1)
        C = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x); // (x2 - x1)(y3 - y1) - (y2 - y1)(x3 - x1)
        D = -1 * (A * point1.x + B * point1.y + C * point1.z); // -(A * x1 + B * y1 + C * z1)

        // Measure distance between every point and fitted plane
        std::unordered_set<int> inliersTemp;
        for (auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
            float d = fabs(A * (*it).x + B * (*it).y + C * (*it).z + D) / sqrt(A * A + B * B + C * C); // |A*x+B*y+C*z+D|/(A^2+B^2+C^2)
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceTol) {
                inliersTemp.insert(it - cloud->begin());
            }
        }
        if (inliersTemp.size() > inliersResult.size()) {
            inliersResult = inliersTemp;
        }
    }

    // Return indicies of inliers from fitted plane with most inliers
    return inliersResult;
}


int main ()
{
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // Change the max iteration and distance tolerance arguments for Ransac function
    // std::unordered_set<int> inliers = RansacLine1(cloud, 50, 0.5);
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
