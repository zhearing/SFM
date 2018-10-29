// Copyright 2018 Zeyu Zhong
// Lincese(MIT)
// Author: Zeyu Zhong
// Date: 2018.5.7

#include "../src/essential.h"
#include <string>
#include <opencv2/calib3d/calib3d.hpp>

using cv::FileStorage;

const int THR_D = 40;

void essential::getIntrinsic() {
    std::string filename = "../../src/Camera/Kseq8.xml";

    FileStorage fs;
    fs.open(filename, FileStorage::READ);

    if (!fs.isOpened())
        std::cerr << "Failed to open " << filename << std::endl;

    fs["K"] >> K;
    K.convertTo(K, CV_64FC1);
//    std::cout<<K<<std::endl;
}

void essential::computeEssentialMat(vector <cv::Point2f> iF1, vector <cv::Point2f> iF2) {
    F = findFundamentalMat(iF1, iF2, cv::FM_RANSAC, 1.3, 0.99);
//    std::cout<<" Fundamental Matrix " << F <<std::endl;

    // Computing the Essential matrix
    E = K.t() * F * K;
//    std::cout<<"Essential Matrix"<<E<<std::endl;
}

void essential::computePose() {
    cv::SVD svd(E);

    Mat W = (cv::Mat_<double>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
    Mat Z = (cv::Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 0);

    //  Two possible rotation matrix

    R1 = (svd.u) * (W) * (svd.vt);
    R2 = (svd.u) * (W.t()) * (svd.vt);

    //  Two possible translation matrix
    t1 = (svd.u).col(2);
    t2 = -(svd.u).col(2);

    //  Check the determinant
    if (determinant(R1) < 0.0)
        R1 = -R1;

    if (determinant(R2) < 0.0)
        R2 = -R2;

    P0 = cv::Mat::eye(3, 4, CV_64FC1);
    P1 = cv::Mat(3, 4, CV_64FC1, cv::Scalar(0));
    P2 = cv::Mat(3, 4, CV_64FC1, cv::Scalar(0));
    P3 = cv::Mat(3, 4, CV_64FC1, cv::Scalar(0));
    P4 = cv::Mat(3, 4, CV_64FC1, cv::Scalar(0));

    // R1 with t1, R1 with t2, R2 with t1, R2 with t2;
    R1.copyTo(P1(cv::Range(0, 3), cv::Range(0, 3)));
    t1.copyTo(P1.col(3));
    R1.copyTo(P2(cv::Range(0, 3), cv::Range(0, 3)));
    t2.copyTo(P2.col(3));

    R2.copyTo(P3(cv::Range(0, 3), cv::Range(0, 3)));
    t1.copyTo(P3.col(3));
    R2.copyTo(P4(cv::Range(0, 3), cv::Range(0, 3)));
    t2.copyTo(P4.col(3));

    P0 = K * P0;
    P1 = K * P1;
    P2 = K * P2;
    P3 = K * P3;
    P4 = K * P4;
}

Mat essential::sign(Mat A) {
    Mat S(1, A.cols, CV_64FC1, cv::Scalar(0));

    for (int i = 0; i < A.cols; i++) {
        double x = A.at<double>(0, i);
        double s = (x > 0) ? 1 : ((x < 0) ? -1 : 0);
        S.at<double>(0, i) = s;
    }

    return S;
}

void essential::get_valid_3d(Mat P, Mat chiral, Mat X, Mat R, Mat t) {
    P.copyTo(P2c);
    R.copyTo(R2c);
    t.copyTo(t2c);

    for (int i = 0; i < chiral.cols; i++) {
        if (chiral.at<double>(0, i) == 2.0) {
            Mat c = X.col(i);
            double x = c.at<double>(0) / c.at<double>(3);
            double y = c.at<double>(1) / c.at<double>(3);
            double z = c.at<double>(2) / c.at<double>(3);

            // filtering only the points which are not too far from camera
            if (z > 0 && z < THR_D) {
                cv::Point3f pt;
                pt.x = x;
                pt.y = y;
                pt.z = z;
                xReconstructed.push_back(pt);
            }
        }
    }
}


void essential::check_chirality(Mat Xn1, Mat Xn2, Mat Xn3, Mat Xn4) {
    Xn1.convertTo(Xn1, CV_64F);
    Xn2.convertTo(Xn2, CV_64F);
    Xn3.convertTo(Xn3, CV_64F);
    Xn4.convertTo(Xn4, CV_64F);

    // Projecting to the first camera K[I|0]
    Mat ax1 = P0 * Xn1.t();
    Mat ax2 = P0 * Xn2.t();
    Mat ax3 = P0 * Xn3.t();
    Mat ax4 = P0 * Xn4.t();

    // Projecting to other camera's K[Ri|ti], i = {1, 2, 3, 4}
    Mat bx1 = P1 * Xn1.t();
    Mat bx2 = P2 * Xn2.t();
    Mat bx3 = P3 * Xn3.t();
    Mat bx4 = P4 * Xn4.t();

    // Now check the chirality for each projections
    Mat chiral1 = sign(ax1.row(2)).mul(sign(Xn1.t().row(3))) + sign(bx1.row(2)).mul(sign(Xn1.t().row(3)));
    Mat chiral2 = sign(ax2.row(2)).mul(sign(Xn2.t().row(3))) + sign(bx2.row(2)).mul(sign(Xn2.t().row(3)));
    Mat chiral3 = sign(ax3.row(2)).mul(sign(Xn3.t().row(3))) + sign(bx3.row(2)).mul(sign(Xn3.t().row(3)));
    Mat chiral4 = sign(ax4.row(2)).mul(sign(Xn4.t().row(3))) + sign(bx4.row(2)).mul(sign(Xn4.t().row(3)));

    Mat chiral_sum(1, 4, CV_64F);
    cv::Scalar sc1 = sum(chiral1);
    cv::Scalar sc2 = sum(chiral2);
    cv::Scalar sc3 = sum(chiral3);
    cv::Scalar sc4 = sum(chiral4);
    chiral_sum.at<double>(0, 0) = static_cast<double>(sc1(0));
    chiral_sum.at<double>(0, 1) = static_cast<double>(sc2(0));
    chiral_sum.at<double>(0, 2) = static_cast<double>(sc3(0));
    chiral_sum.at<double>(0, 3) = static_cast<double>(sc4(0));

    double mx, mn;
    cv::Point minLoc, maxLoc;
    minMaxLoc(chiral_sum, &mn, &mx, &minLoc, &maxLoc);

    switch (maxLoc.x) {
        case 0:
            get_valid_3d(P1, chiral1, Xn1.t(), R1, t1);
            break;
        case 1:
            get_valid_3d(P2, chiral2, Xn2.t(), R1, t2);
            break;
        case 2:
            get_valid_3d(P3, chiral3, Xn3.t(), R2, t1);
            break;
        case 3:
            get_valid_3d(P4, chiral4, Xn4.t(), R2, t2);
            break;
    }
}
