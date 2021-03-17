/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>
#include "../ukf.h"

struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {}
};

struct Vect3
{

    double x, y, z;

    Vect3(double setX, double setY, double setZ)
        : x(setX), y(setY), z(setZ)
    {}

    Vect3 operator+(const Vect3& vec)
    {
        Vect3 result(x + vec.x, y + vec.y, z + vec.z);
        return result;
    }
};

enum CameraAngle
{
    XY, TopDown, Side, FPS
};

struct accuation
{
    long long time_us;
    float acceleration;
    float steering;

    accuation(long long t, float acc, float s)
        : time_us(t), acceleration(acc), steering(s)
    {}
};

struct Car
{

    // units in meters
    Vect3 position, dimensions;
    Eigen::Quaternionf orientation;
    std::string name;
    Color color;
    float velocity;
    float angle;
    float acceleration;
    float steering;
    // distance between front of vehicle and center of gravity
    float Lf;

    UKF ukf;

    //accuation instructions
    std::vector<accuation> instructions;
    int accuateIndex;

    double sinNegTheta;
    double cosNegTheta;

    Car()
        : position(Vect3(0,0,0)), dimensions(Vect3(0,0,0)), color(Color(0,0,0))
    {}

    Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity, float setAngle, float setLf, std::string setName)
        : position(setPosition), dimensions(setDimensions), color(setColor), velocity(setVelocity), angle(setAngle), Lf(setLf), name(setName)
    {
        orientation = getQuaternion(angle);
        acceleration = 0;
        steering = 0;
        accuateIndex = -1;

        sinNegTheta = sin(-angle);
        cosNegTheta = cos(-angle);
    }

    // angle around z axis
    Eigen::Quaternionf getQuaternion(float theta)
    {
        Eigen::Matrix3f rotation_mat;
          rotation_mat <<
          cos(theta), -sin(theta), 0,
        sin(theta),  cos(theta), 0,
        0, 			 0, 		 1;

        Eigen::Quaternionf q(rotation_mat);
        return q;
    }

    void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
    {
        // render bottom of car
        viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
        viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name+"frame");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"frame");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"frame");


        // render top of car
        viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name + "Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "Top");
        viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Topframe");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"Topframe");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"Topframe");
    }

    void setAcceleration(float setAcc)
    {
        acceleration = setAcc;
    }

    void setSteering(float setSteer)
    {
        steering = setSteer;
    }

    void setInstructions(std::vector<accuation> setIn)
    {
        for(accuation a : setIn)
            instructions.push_back(a);
    }

    void setUKF(UKF tracker)
    {
        ukf = tracker;
    }

    void move(float dt, int time_us)
    {

        if(instructions.size() > 0 && accuateIndex < (int)instructions.size()-1)
        {
            if(time_us >= instructions[accuateIndex+1].time_us)
            {
                setAcceleration(instructions[accuateIndex+1].acceleration);
                setSteering(instructions[accuateIndex+1].steering);
                accuateIndex++;
            }
        }

        position.x += velocity * cos(angle) * dt;
        position.y += velocity * sin(angle) * dt;
        angle += velocity*steering*dt/Lf;
        orientation = getQuaternion(angle);
        velocity += acceleration*dt;

        sinNegTheta = sin(-angle);
        cosNegTheta = cos(-angle);
    }

    // collision helper function
    bool inbetween(double point, double center, double range)
    {
        return (center - range <= point) && (center + range >= point);
    }

    bool checkCollision(Vect3 point)
    {
        // check collision for rotated car
        double xPrime = ((point.x-position.x) * cosNegTheta - (point.y-position.y) * sinNegTheta)+position.x;
        double yPrime = ((point.y-position.y) * cosNegTheta + (point.x-position.x) * sinNegTheta)+position.y;

        return (inbetween(xPrime, position.x, dimensions.x / 2) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
            (inbetween(xPrime, position.x, dimensions.x / 4) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));

    }
};

void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1, 1, 1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1, -1, -1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1, 0, 0), float opacity = 1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color = Color(1, 0, 0), float opacity = 1);

#endif
