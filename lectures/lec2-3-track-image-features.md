# Lesson 2-3 Tracking Image Features

This lesson will show how to identify and track reliable and stable features through a sequence of images.
1. Locate keypionts (point of interest)



### I. Intensity Gradient and Filtering

The majority of the keypoint types is the distribution of brightness information over the image. High intensity gradient indicates the brightness changes rapidly, and those places might therefore be suitable keypoints.

<img src="media/intensity-gradient.png" width="600" height="450" />

Mathematically, the gradient is the partial derivative of the image intensity into both *x* and *y* direction. The **approximation of gradient** would be the intensity differences between neighboring pixels, divided by the distance between those pixels. We can also compute the **direction** as well as the **magnitude** of the intensity gradient vector.

<img src="media/intensity-gradient-calculation.png" width="600" height="300" />

`Sobel` operator is one of the most famous approaches to compute the gradient. They are 3x3 kernels, as small integer-valued filters.

<img src="media/sobel-operator.png" width="600" height="150" />


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

#### Exercise: Computing Intensity Gradient

In the [gradient_sobel.cpp](../Camera/Lesson-4-Tracking-Image-Features/Intensity-Gradient-and-Filtering/gradient_filtering/src/gradient_sobel.cpp), create the sobel kernels and apply the filters to *x* and *y* directions respectively. ([73a5b01](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/73a5b013c6fc2f664c999581902742bd7aa11e35))

<img src="media/sobel-kernel-filter.png" width="800" height="500" />

In the [magnitude_sobel.cpp](../Camera/Lesson-4-Tracking-Image-Features/Intensity-Gradient-and-Filtering/gradient_filtering/src/magnitude_sobel.cpp), the processing pipeline is: convert the image to gray scale -> smooth it using `cv::GaussianBlur()` -> apply `cv::filter2D()` with sobel kernels in both *x* and *y* directions -> calculated the magnitude for each pixel based on the *x*/*y* gradient. ([4d5fc96](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/4d5fc96e4e3ac05f2c064644c433f0f17fa1034f))

<img src="media/sobel-filtered-magnitude.png" width="800" height="250" />



### II. Keypoint Tracking

#### Harris Corner Detection

The idea of keypoint detection is to detect a unique structure in an image that can be precisely located in both coordinate directions. As discussed in the previous section, corners are ideally suited for this purpose. In order to locate a corner, we consider how the content of the window would change when shifting it by a small amount. Such change is described by the *sum of squared differences (SSD)*.

<img src="media/locate-a-corner.png" width="600" height="250" />

A *covariance matrix* `Hw` is part of the calculation result of that change. The matrix `Hw` can be visualized as an eclipse, whose axis length and direction are given by its **eigenvalues** and **eigenvectors**. For *Harris Detector*, we can derive a corner response measure at every pixel location with the factor *k* being an empirical constant (0.04 ~ 0.06).

<img src="media/harris-corner-response.png" width="800" height="300" />

#### Exercise:

After getting Harris corner response, it is time to perform a **non-maxima suppression (NMS)** to:
1. ensure the pixel with maximum corner response in a local neighborhood
2. prevent corners from being too close to each other

In the [cornerness_harris.cpp](../Camera/Lesson-4-Tracking-Image-Features/Harris-Corner-Detection/cornerness_harris/src/cornerness_harris.cpp), first, the Harris Corner Response matrix is calculated. Then we locate a local maxima in the response matrix and perform NMS to it. ([2776d68](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/2776d68e13deaf24e7aebf0924803ed07e2069db))

<img src="media/harris-corner-detection-exercise.png" width="800" height="250" />



### III.


### IV.

