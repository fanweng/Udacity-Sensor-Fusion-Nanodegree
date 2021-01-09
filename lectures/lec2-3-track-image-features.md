# Lesson 2-3 Tracking Image Features

This lesson will show how to identify and track reliable and stable features through a sequence of images.
1. Locate keypionts (point of interest)



### I. Intensity Gradient and Filtering

The majority of the keypoint types is the distribution of brightness information over the image. High intensity gradient indicates the brightness changes rapidly, and those places might therefore be suitable keypoints.

<img src="media/intensity-gradient.png" width="600" height="450" />

Mathematically, the gradient is the partial derivative of the image intensity into both *x* and *y* direction. The **approximation of gradient** would be the intensity differences between neighboring pixels, divided by the distance between those pixels. We can also compute the **direction** as well as the **magnitude** of the intensity gradient vector.

<img src="media/intensity-gradient-calculation.png" width="600" height="300" />

#### Gaussian Filtering

Before computing the intensity gradient, we should apply noise filtering to get an accurate result. **Gaussian filter** is shifted over the image and combined with the intensity values beneath it. Two parameters can be adjusted:

1. **Standard deviation:** controls the spatial extension of the filter in the image plane. Larger the standard deviation, wider the area which is covered by the filter

2. **[Kernel size](https://docs.opencv.org/3.4/d4/dbd/tutorial_filter_2d.html):** defines the number of pixels around the center location will contribute to the smoothing operation

<img src="media/gaussian-filter.png" width="800" height="250" />

Gaussian filtering works by assigning each pixel a weighted sum of the surrounding pixels based on the heigh of Gaussian curve at each point. The largest contribution will come from the center pixel itself. The steps like:

1. Create a filter kernel with the desired properties (e.g. Gaussian smoothing or edge detection)
2. Define the anchor point within the kernel (usually the center position) and place it on top of the first pixel of the image
3. Compute the sum of the products of kernel coefficients with the corresponding image pixel values beneath
4. Place the result to the location of the kernel anchor in the input image
5. Repeat the process for all pixels over the entire image.

#### Exercise: Image Filters and Gaussian Smoothing

In the [gaussian_smoothing.cpp](../Camera/Lesson-4-Tracking-Image-Features/Intensity-Gradient-and-Filtering/gradient_filtering/src/gaussian_smoothing.cpp), before applying `cv::filter2D()`, the kernel coefficients must be normalized. ([6854a85](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/6854a85edb63e91aa7677477af159d3963b0d2f7))

<img src="media/gaussian-kernel-coefficients-exercise.png" width="800" height="250" />



### II.


### III.


### IV.

