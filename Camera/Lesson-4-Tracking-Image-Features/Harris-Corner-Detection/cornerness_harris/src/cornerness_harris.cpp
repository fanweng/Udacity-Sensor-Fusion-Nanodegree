#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    // locate local maxima in the Harris response matrix
    vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0;
    for (int j = 0; j < dst_norm.rows; j++)
    {
        for (int i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            {
                cv::KeyPoint newKeypoint;
                newKeypoint.pt = cv::Point2f(i, j);
                newKeypoint.size = 2 * apertureSize;
                newKeypoint.response = response;

                // perform a non-maximum suppression (NMS) in a local neighborhood around each maximum
                bool isOverlapped = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double overlap = cv::KeyPoint::overlap(newKeypoint, *it);
                    if (overlap > maxOverlap)
                    {
                        isOverlapped = true;
                        if (newKeypoint.response > (*it).response)
                        {
                            *it = newKeypoint; // replace the keypoint with a higher response one
                            break;
                        }
                    }
                }

                // add the new keypoint which isn't consider to have overlap with the keypoints already stored in the list
                if (!isOverlapped)
                {
                    keypoints.push_back(newKeypoint);
                }
            }
        }
    }

    // visualize results
    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 1);
    cv::Mat imgCorners = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, imgCorners, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, imgCorners);
    cv::waitKey(0);
}

int main()
{
    cornernessHarris();
}