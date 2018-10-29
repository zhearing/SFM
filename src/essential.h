// Copyright 2018 Zeyu Zhong
// Lincese(MIT)
// Author: Zeyu Zhong
// Date: 2018.5.7

#ifndef SRC_ESSENTIAL_H_
#define SRC_ESSENTIAL_H_

#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using cv::Mat;
using std::vector;

class essential {
 public:
    void getIntrinsic();

    void computeEssentialMat(vector <cv::Point2f>, vector <cv::Point2f>);

    void computePose();

    void check_chirality(Mat, Mat, Mat, Mat);

    Mat P0, P1, P2, P3, P4;
    Mat P2c, R2c, t2c;
    vector <cv::Point3d> xReconstructed;

 protected:
    Mat K, F, E, R1, R2, t1, t2;

 private:
    void get_valid_3d(Mat, Mat, Mat, Mat, Mat);

    Mat sign(Mat);
};

#endif  // SRC_ESSENTIAL_H_
