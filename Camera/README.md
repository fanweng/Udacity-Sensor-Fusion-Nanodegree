# Camera Projects

This repo contains lesson-wise exercises and corresponding solutions for Udacity's Sensor Fusion ND. 



### I. List of Lesson-wise Exercises

- Original source code of lesson exercises is available here - https://github.com/udacity/SFND_Camera
    * Lesson 2: Autonomous Vehicles and Computer Vision ([Lecture/Exercise notes](../lectures/lec2-1-av-and-opencv.md))
        + The OpenCV Library
    * Lesson 3: Engineering a Collision Detection System ([Lecture/Exercise notes](../lectures/lec2-2-collision-detection-system.md))
        + Estimating TTC with Camera
        + Estimating TTC with Lidar
    * Lesson 4: Tracking Image Features ([Lecture/Exercise notes](../lectures/lec2-3-track-image-features.md))
        + Descriptor Matching
        + Gradient-based vs. Binary Descriptors
        + Harris Corner Detection
        + Intensity Gradient and Filtering
        + Overview of Popular Keypoint Detectors
    * Lesson 6: Combining Camera and Lidar ([Lecture/Exercise notes](../lectures/lec2-4-combine-camera-lidar.md))
        + Creating 3D-Objects
        + Lidar-to-Camera Point Projection
        + Object Detection with YOLO

- Projects
    * Lesson 5: Starter code for "Project: Camera Based 2D Feature Tracking" is available here - https://github.com/udacity/SFND_2D_Feature_Tracking
        + Project report ([HERE](./Lesson-5-Project-2D-Feature-Tracking/README.md))
    * Lesson 7: Starter code for "Project: Track an Object in 3D Space" is available here - https://github.com/udacity/SFND_3D_Object_Tracking
        + Project report ([HERE](./Lesson-7-Project-3D-Object-Tracking/README.md))



### II. Dependencies for Running Locally

1. cmake >= 2.8
    * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1
    * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
    * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
    * Install the OpenCV dependencies as well as OpenCV Contrib for extra modules, then build from the source. Instruction [HERE](http://techawarey.com/programming/install-opencv-c-c-in-ubuntu-18-04-lts-step-by-step-guide/).
    * **NOTE:** the license issue still might occur when using non-free algorithms, in that case, please use the online workspace provided by Udacity.
```bash
# compiler
$ sudo apt-get install build-essential
# required
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libcanberra-gtk-module
# optional
$ sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

$ git clone https://github.com/opencv/opencv.git
$ git clone https://github.com/opencv/opencv_contrib.git <opencv_contrib_dir>
$ cd ./opencv
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=<opencv_contrib_dir>/modules -DOPENCV_ENABLE_NONFREE=ON ..
$ make -j4
$ sudo make install
```

4. gcc/g++ >= 5.4 
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)



### III. Build Instructions

Go to the top level directory for an exercise or a project where the `CMakeLists.txt` file locates, and run the following commands on the Linux terminal:
```bash
$ mkdir build && cd build
$ cmake ..
$ make
$ ./<Executable_File_Name>
```



## IV. Pushing Large Files (>100MB) [Optional]

The file `Combining Camera and Lidar/Object Detection with YOLO/detect_objects/dat/yolo/yolov3.weights` is 236.52 MB; this exceeds GitHub's file size limit of 100.00 MB

Github blocks the files that have size>100MB, while pushing them. To push large files, Git provides an option called Git Large File Storage (LFS). See the instructions at [https://git-lfs.github.com/](https://git-lfs.github.com/) to use Git LFS. See [http://git.io/iEPt8g](http://git.io/iEPt8g) for more information.

You can push large file only to unprotected remote branches. Master branch is by default protected. Read more [here](https://docs.github.com/en/github/administering-a-repository/about-protected-branches). Therefore, you'll have to push to a new unprotected branch, and later, merge it with the remote master. Use the commands below:

* Create a new local branch
```
git checkout -b <local_branch_name>
```

* Write the solution to your exercise. 

* Install Git Large File Storage (LFS). For MacOS, use
```
brew install git-lfs
git lfs install
git lfs track "<Large_file_name_if_any>"
git add .gitattributes
git add <path_to_the_Large_file_if_any>
git config --global lfs.contenttype 0

```

* Add the modified files to the index area, and commit the changes
```
git add . --all   
git commit -m "your comment"
```
* To push the current local branch and set the remote as upstream:
```
git push --set-upstream origin <local_branch_name>
```

* Next, create a PR and merge the new branch with the remote master.