//
//  dataStructures.h
//  OpenCVTest
//
//  Created by Andreas Haja on 01.04.19.
//  Copyright © 2019 Andreas Haja. All rights reserved.
//

#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};


#endif /* dataStructures_h */
