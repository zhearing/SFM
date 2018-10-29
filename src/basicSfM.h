// Copyright 2018 Zeyu Zhong
// Lincese(MIT)
// Author: Zeyu Zhong
// Date: 2018.5.7

#ifndef SRC_BASICSFM_H_
#define SRC_BASICSFM_H_

#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using cv::Mat;
using std::vector;

class basicSfM {
 public:
    void algorithm_sparse3d();

 private:
    void reconstruct_sparse3d(vector <Mat>);

    vector <cv::Point3d> xR;
    Mat iP1, iP2;
};

#endif  // SRC_BASICSFM_H_
