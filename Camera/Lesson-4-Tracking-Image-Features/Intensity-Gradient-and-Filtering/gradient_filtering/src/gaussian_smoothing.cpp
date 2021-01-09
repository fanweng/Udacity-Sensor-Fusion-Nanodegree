#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gaussianSmoothing1()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // create filter kernel
    int gauss_size = 25;
    float gauss_data[gauss_size] = {1, 4, 7, 4, 1,
                                    4, 16, 26, 16, 4,
                                    7, 26, 41, 26, 7,
                                    4, 16, 26, 16, 4,
                                    1, 4, 7, 4, 1};
    cv::Mat kernel = cv::Mat(5, 5, CV_32F, gauss_data);

    // calculate the normalized kernel coefficients
    float sum = accumulate(gauss_data, gauss_data + gauss_size, 0.0);
    for (int i = 0; i < gauss_size; i++) {
        gauss_data[i] /= sum;
    }

    // apply filter
    cv::Mat result;
    cv::filter2D(img, result, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, result);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    gaussianSmoothing1();
}