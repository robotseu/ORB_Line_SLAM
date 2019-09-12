/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Converter.h"

namespace ORB_SLAM2
{

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}



cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toInvCvMat(const cv::Mat &T_cvMat)
{
    cv::Mat T_inv = cv::Mat::eye(4,4,T_cvMat.type());
    cv::Mat R = T_cvMat.rowRange(0,3).colRange(0,3);
    cv::Mat t = T_cvMat.rowRange(0,3).col(3);
    cv::Mat R_trans = R.t();
    cv::Mat t_inv = -R_trans*t;

    R_trans.copyTo(T_inv.rowRange(0,3).colRange(0,3));
    t_inv.copyTo(T_inv.rowRange(0,3).col(3));
    return T_inv.clone();
}

cv::Mat Converter::toInvCvMat(const Eigen::Matrix<double,4,4> &T_eigenMtrx)
{
    return toInvCvMat(toCvMat(T_eigenMtrx));

//    cv::Mat T_inv = cv::Mat::eye(4,4,CV_32F);
//    Eigen::Matrix3d Reigen = T_eigenMtrx.block<3,3>(0,0);
//    Eigen::Matrix<double,3,1> teigen = T_eigenMtrx.block<3,1>(0,3);
//    cv::Mat R = Converter::toCvMat(Reigen);
//    cv::Mat t = Converter::toCvMat(teigen);
//    cv::Mat R_trans = R.t();
//    cv::Mat t_inv = -R_trans*t;

//    R_trans.copyTo(T_inv.rowRange(0,3).colRange(0,3));
//    t_inv.copyTo(T_inv.rowRange(0,3).col(3));
//    return T_inv.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

Eigen::Matrix<double,4,4> Converter::toMatrix4d(const cv::Mat &T)
{
    Eigen::Matrix<double,4,4> M;
    M << T.at<float>(0, 0), T.at<float>(0, 1), T.at<float>(0, 2), T.at<float>(0, 3),
    T.at<float>(1, 0), T.at<float>(1, 1), T.at<float>(1, 2), T.at<float>(1, 3),
    T.at<float>(2, 0), T.at<float>(2, 1), T.at<float>(2, 2), T.at<float>(2, 3),
                    0,                 0,                 0,                 1;
    return M;
}

Eigen::Matrix<double,4,4> Converter::toInvMatrix4d(const Eigen::Matrix<double,4,4> &T_eigenMtrx)
{
    Eigen::Matrix<double,4,4> T_inv(Eigen::Matrix4d::Identity());
    Eigen::Matrix<double,3,3> R = T_eigenMtrx.block<3,3>(0,0);
    Eigen::Matrix<double,3,1> t = T_eigenMtrx.block<3,1>(0,3);
    T_inv.block<3,3>(0,0) = R.transpose();
    T_inv.block<3,1>(0,3) = -R.transpose()*t;
    return T_inv;
}

Eigen::Matrix<double,4,4> Converter::toInvMatrix4d(const cv::Mat &T_cvMat)
{
    return toInvMatrix4d(toMatrix4d(T_cvMat));

//    Eigen::Matrix<double,4,4> T_inv(Eigen::Matrix4d::Identity());
//    Eigen::Matrix<double,3,3> R;
//    Eigen::Matrix<double,3,1> t;
//    R <<    T_cvMat.at<float>(0, 0), T_cvMat.at<float>(0, 1), T_cvMat.at<float>(0, 2),
//            T_cvMat.at<float>(1, 0), T_cvMat.at<float>(1, 1), T_cvMat.at<float>(1, 2),
//            T_cvMat.at<float>(2, 0), T_cvMat.at<float>(2, 1), T_cvMat.at<float>(2, 2);
//    t <<    T_cvMat.at<float>(0, 3), T_cvMat.at<float>(1, 3), T_cvMat.at<float>(2, 3);
//    T_inv.block<3,3>(0,0) = R.transpose();
//    T_inv.block<3,1>(0,3) = -R.transpose()*t;
//    return T_inv;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

void Converter::RmatOfQuat(cv::Mat &M, const cv::Mat &q)
{
    Eigen::Quaterniond _q(q.at<float>(0,3), q.at<float>(0,0), q.at<float>(0,1), q.at<float>(0,2));
    Eigen::Matrix<double,3,3> _m = _q.toRotationMatrix();
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
    M.at<float>(i,j) = _m(i,j);
}

} //namespace ORB_SLAM
