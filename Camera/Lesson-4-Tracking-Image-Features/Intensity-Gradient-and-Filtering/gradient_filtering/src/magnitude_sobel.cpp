#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    cv::Mat imgBlurred;
    int kernelSize = 5;
    double stdDeviation = 2.0;
    cv::GaussianBlur(imgGray, imgBlurred, cv::Size(kernelSize, kernelSize), stdDeviation, stdDeviation, cv::BORDER_DEFAULT);

    // create filter kernels using the cv::Mat datatype both for x and y
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2,
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    float sobel_y[9] = {-1, -2, -1,
                         0,  0,  0,
                        +1, +2, +1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);

    // apply filter using the OpenCv function filter2D()
    cv::Mat imgFilteredX;
    cv::filter2D(imgBlurred, imgFilteredX, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    cv::Mat imgFilteredY;
    cv::filter2D(imgBlurred, imgFilteredY, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // compute magnitude image based on the equation presented in the lesson
    cv::Mat imgMag = imgGray.clone();
    for (int i = 0; i < imgMag.rows-1; i++)
    {
        for (int j = 0; j < imgMag.cols-1; j++)
        {
            imgMag.at<unsigned char>(i, j) = sqrt(pow(imgFilteredX.at<unsigned char>(i, j), 2) + pow(imgFilteredY.at<unsigned char>(i, j), 2));
        }
    }

    // show result
    string windowName = "Sobel Filtered Magnitude";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, imgMag);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}