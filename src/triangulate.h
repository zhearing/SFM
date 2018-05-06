// Copyright 2018 Zeyu Zhong
// Lincese(MIT)
// Author: Zeyu Zhong
// Date: 2018.5.7

#ifndef SRC_TRIANGULATE_H_
#define SRC_TRIANGULATE_H_

#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using std::vector;
using namespace cv;

class triangulate {
 public:
    Mat triangulate_points(vector<Point2f>, vector<Point2f>, float, Mat, Mat);
};

#endif  // SRC_TRIANGULATE_H_
