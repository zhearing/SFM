// Copyright 2018 Zeyu Zhong
// Lincese(MIT)
// Author: Zeyu Zhong
// Date: 2018.5.7

#include "../src/basicSfM.h"
#include <iterator>
#include <string>
#include "../src/matches.h"
#include "../src/essential.h"
#include "../src/triangulate.h"

void basicSfM :: reconstruct_sparse3d(vector<Mat> images) {
//    std::cout<<"Step1. Compute correspondance match between images "<<std::endl;
    matches Mt;

    Mat image1 = images[0];
    Mat image2 = images[1];

    Mt.computeCorrespondances(image1, image2, false);
    vector<cv::Point2f> iF1 = Mt.i1_features;

    for (int i = 2; i < images.size(); i++) {
        images[i].copyTo(image2);
        Mt.i1_features = Mt.i2_features;
        Mt.computeCorrespondances(image1, image2, true);
    }

    vector<cv::Point2f> iF2 = Mt.i2_features;
    Mat tracks = Mt.drawCorrespondances();

    cv::imshow("tracks", tracks);
    cv::imwrite("tracks.jpg", tracks);
    cv::waitKey(0);

//    std::cout<<"Step2. Compute  Essential Matrix "<<std::endl;

    essential Est;
    Est.getIntrinsic();
    Est.computeEssentialMat(iF1, iF2);

//    std::cout<<"Step3. Decomposition of E into four possible camera poses"<<std::endl;

    Est.computePose();

//    std::cout<<"Step4. Reconstruct for possible Rt matices using Triangulation" <<std::endl;
    triangulate tr; float scale = 1.0;

    Mat Xn1 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P1);
    Mat Xn2 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P2);
    Mat Xn3 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P3);
    Mat Xn4 = tr.triangulate_points(iF1, iF2, scale, Est.P0, Est.P4);

//    std::cout<<"Step5. Check the chirality to validate the reconstruction"<<std::endl;
    Est.check_chirality(Xn1, Xn2, Xn3, Xn4);
    vector<cv::Point3d> xR = Est.xReconstructed;
    vector<cv::Point3d> :: iterator it = xR.begin();

    while (it != xR.end()) {
        std::cout << it->x << " " << it->y << " " << it->z << std::endl;
        it++;
    }
    (Est.P2c).copyTo(iP1);
}

void basicSfM :: algorithm_sparse3d() {
    vector<Mat> images;
    for (int i = 15; i <= 19; i++) {
        std::stringstream ss;
        ss << i;
        std::string iPath = "../../images/" + ss.str() + ".pgm";
        Mat image = cv::imread(iPath, 1);
        images.push_back(image);
    }
    reconstruct_sparse3d(images);
}
