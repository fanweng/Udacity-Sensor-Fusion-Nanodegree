#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace std;

void createMatrix1()
{
    // create matrix
    int nrows = 480, ncols = 640;
    cv::Mat m1_8u;
    m1_8u.create(nrows, ncols, CV_8UC1); // two-channel matrix with 8bit unsigned elements
    m1_8u.setTo(cv::Scalar(255));        // white

    // STUDENT TASK :
    // Create a variable of type cv::Mat* named m3_8u which has three channels with a
    // depth of 8bit per channel. Then, set the first channel to 255 and display the result.
    cv::Mat m3_8u;
    m3_8u.create(nrows, ncols, CV_8UC3); // three-channel matrix with 8bit unsigned elements
    m3_8u.setTo(cv::Scalar(255, 0, 0));  // blue

    // show result
    string windowName = "First steps in OpenCV (m1_8u)";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, m1_8u);
    cv::waitKey(0); // wait for keyboard input before continuing

    // STUDENT TASK :
    // Display the results from the STUDENT TASK above
    windowName = "First steps in OpenCV (m3_8u)";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, m3_8u);
    cv::waitKey(0); // wait for keyboard input before continuing
}


int main()
{
    createMatrix1();
    return 0;
}