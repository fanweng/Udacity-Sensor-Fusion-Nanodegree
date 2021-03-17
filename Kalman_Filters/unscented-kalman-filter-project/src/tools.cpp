#include <iostream>
#include <random>
#include "tools.h"

using namespace std;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

double Tools::noise(double stddev, long long seedNum)
{
    mt19937::result_type seed = seedNum;
    auto dist = std::bind(std::normal_distribution<double>{0, stddev}, std::mt19937(seed));
    return dist();
}

// sense where a car is located using lidar measurement
lmarker Tools::lidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize)
{
    MeasurementPackage meas_package;
    meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);

    lmarker marker = lmarker(car.position.x + noise(0.15,timestamp), car.position.y + noise(0.15,timestamp+1));
    if(visualize)
        viewer->addSphere(pcl::PointXYZ(marker.x,marker.y,3.0),0.5, 1, 0, 0,car.name+"_lmarker");

    meas_package.raw_measurements_ << marker.x, marker.y;
    meas_package.timestamp_ = timestamp;

    car.ukf.ProcessMeasurement(meas_package);

    return marker;
}

// sense where a car is located using radar measurement
rmarker Tools::radarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize)
{
    double rho = sqrt((car.position.x-ego.position.x)*(car.position.x-ego.position.x)+(car.position.y-ego.position.y)*(car.position.y-ego.position.y));
    double phi = atan2(car.position.y-ego.position.y,car.position.x-ego.position.x);
    double rho_dot = (car.velocity*cos(car.angle)*rho*cos(phi) + car.velocity*sin(car.angle)*rho*sin(phi))/rho;

    rmarker marker = rmarker(rho+noise(0.3,timestamp+2), phi+noise(0.03,timestamp+3), rho_dot+noise(0.3,timestamp+4));
    if(visualize)
    {
        viewer->addLine(pcl::PointXYZ(ego.position.x, ego.position.y, 3.0), pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi), 3.0), 1, 0, 1, car.name+"_rho");
        viewer->addArrow(pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi), 3.0), pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi)+marker.rho_dot*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi)+marker.rho_dot*sin(marker.phi), 3.0), 1, 0, 1, car.name+"_rho_dot");
    }

    MeasurementPackage meas_package;
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    meas_package.raw_measurements_ << marker.rho, marker.phi, marker.rho_dot;
    meas_package.timestamp_ = timestamp;

    car.ukf.ProcessMeasurement(meas_package);

    return marker;
}

// Show UKF tracking and also allow showing predicted future path
// double time:: time ahead in the future to predict
// int steps:: how many steps to show between present and time and future time
void Tools::ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps)
{
    UKF ukf = car.ukf;
    viewer->addSphere(pcl::PointXYZ(ukf.x_[0],ukf.x_[1],3.5), 0.5, 0, 1, 0,car.name+"_ukf");
    viewer->addArrow(pcl::PointXYZ(ukf.x_[0], ukf.x_[1],3.5), pcl::PointXYZ(ukf.x_[0]+ukf.x_[2]*cos(ukf.x_[3]),ukf.x_[1]+ukf.x_[2]*sin(ukf.x_[3]),3.5), 0, 1, 0, car.name+"_ukf_vel");
    if(time > 0)
    {
        double dt = time/steps;
        double ct = dt;
        while(ct <= time)
        {
            ukf.Prediction(dt);
            viewer->addSphere(pcl::PointXYZ(ukf.x_[0],ukf.x_[1],3.5), 0.5, 0, 1, 0,car.name+"_ukf"+std::to_string(ct));
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-0.8*(ct/time), car.name+"_ukf"+std::to_string(ct));
            //viewer->addArrow(pcl::PointXYZ(ukf.x_[0], ukf.x_[1],3.5), pcl::PointXYZ(ukf.x_[0]+ukf.x_[2]*cos(ukf.x_[3]),ukf.x_[1]+ukf.x_[2]*sin(ukf.x_[3]),3.5), 0, 1, 0, car.name+"_ukf_vel"+std::to_string(ct));
            //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-0.8*(ct/time), car.name+"_ukf_vel"+std::to_string(ct));
            ct += dt;
        }
    }

}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
            || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

void Tools::savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII (file, *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Tools::loadPcd(std::string file)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  //std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

  return cloud;
}

