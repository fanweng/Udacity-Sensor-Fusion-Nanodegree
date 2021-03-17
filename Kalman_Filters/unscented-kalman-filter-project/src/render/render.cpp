/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"

void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    // units in meters
    double roadLengthAhead = 50.0;
    double roadLengthBehind = -15.0;
    double roadWidth = 12.0;
    double roadHeight = 0.2;

    viewer->addCube(roadLengthBehind, roadLengthAhead, -roadWidth / 2, roadWidth / 2, -roadHeight, 0, .2, .2, .2, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "highwayPavement");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "highwayPavement");
    viewer->addLine(pcl::PointXYZ(roadLengthBehind, -roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead , -roadWidth / 6, 0.01), 1, 1, 0, "line1");
    viewer->addLine(pcl::PointXYZ(roadLengthBehind, roadWidth / 6, 0.01), pcl::PointXYZ(roadLengthAhead, roadWidth / 6, 0.01), 1, 1, 0, "line2");

    // render poles
    // spacing in meters between poles, poles start at x = 0
    double poleSpace = 10;
    // pole distance from road curve
    double poleCurve = 4;
    double poleWidth = 0.5;
    double poleHeight = 3;

    //double distancePos = 7;
    double markerPos = (roadLengthBehind/poleSpace)*poleSpace-distancePos;
    while(markerPos < roadLengthBehind)
        markerPos+=poleSpace;
    int poleIndex = 0;
    while(markerPos <= roadLengthAhead)
    {
        //	left pole
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2+roadWidth/2+poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"l");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"l");
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2+roadWidth/2+poleCurve, poleWidth/2+roadWidth/2+poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"lframe");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"lframe");

        //	right pole
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2-roadWidth/2-poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 1, 0.5, 0, "pole_"+std::to_string(poleIndex)+"r");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "pole_"+std::to_string(poleIndex)+"r");
        viewer->addCube(-poleWidth/2+markerPos, poleWidth/2+markerPos, -poleWidth/2-roadWidth/2-poleCurve, poleWidth/2-roadWidth/2-poleCurve, 0, poleHeight, 0, 0, 0, "pole_"+std::to_string(poleIndex)+"rframe");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "pole_"+std::to_string(poleIndex)+"rframe");

        markerPos+=poleSpace;
        poleIndex++;
    }

}

int countRays = 0;
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

    for (pcl::PointXYZ point : cloud->points)
    {
        viewer->addLine(pcl::PointXYZ(origin.x, origin.y, origin.z), point, 1, 0, 0, "ray" + std::to_string(countRays));
        countRays++;
    }
}

void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    while (countRays)
    {
        countRays--;
        viewer->removeShape("ray" + std::to_string(countRays));
    }
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
{

    viewer->addPointCloud<pcl::PointXYZ>(cloud, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color)
{

    if (color.r == -1)
    {
        // Select color based off of cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
        // Select color based off input value
        viewer->addPointCloud<pcl::PointXYZI>(cloud, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    }

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Draw wire frame box with filled transparent color
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity)
{
    if (opacity > 1.0)
        opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}







