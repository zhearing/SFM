// Copyright 2018 Zeyu Zhong
// Lincese(MIT)
// Author: Zeyu Zhong
// Date: 2018.5.7

#ifndef SRC_MATCHES_H_
#define SRC_MATCHES_H_

#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using cv::Mat;
using std::vector;

class matches {
 public:
    void computeCorrespondances(Mat, Mat, bool);

    Mat drawCorrespondances();

    Mat i1, i2;
    Mat denseflow;
    vector <cv::Point2f> i1_features;
    vector <cv::Point2f> i2_features;
    vector <Mat> i1_colors;

 private:
    void detectfeature();

    void matchFeatures();

    void filterFeatures();

    double qualityLevel;
    double minDistance;
    int blockSize;
    bool useHarrisDetector;
    double k;
    int maxCorners;
};

#endif  // SRC_MATCHES_H_
