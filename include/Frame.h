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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "MapLine.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "LineExtractor.h"

#include <opencv2/opencv.hpp>

// #include <stereoFeatures.h>

#include "Config.h"
#include "LineMatcher.h"
#include "gridStructure.h"

#include <eigen3/Eigen/Core>
using namespace Eigen;

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class MapLine;
class KeyFrame;

class Frame
{
    typedef unsigned int WordId;
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, 
        ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef,
        const float &bf, const float &thDepth);
    // Constructor for stereo cameras with lines
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, 
        Lineextractor* LineextractorLeft, Lineextractor* LineextractorRight, ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef,
        const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);
    // Extract LineFeatures on the image. 0 for left image and 1 for right image.
    void ExtractLine(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Set Information of prevoius frame for track optimization
    void SetprevInformation(cv::Mat _Tcw, Matrix6d _DT_cov, double _err_norm);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    bool isInFrustum_l(MapLine *pML, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();
    // Search a match for each line in the left image to a line in the right image
    void ComputeStereoMatches_Lines(bool initial = true);

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

    // backproject point[u,v] into 3D world coordinate
    Eigen::Vector3d backProjection( const double &u, const double &v, const double &disp );
    // backproject 3D point(camera coodinate) into [u,v] image coordinate
    Eigen::Vector2d projection( const Eigen::Vector3d &P );
    // overlap between obs lineSegment and proj lineSegment
    double lineSegmentOverlap(Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;
    LineVocabulary* mpLinevocabulary;

    // Point Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Line Feature extractor. The right is used only in the stereo case.
    Lineextractor* mpLineextractorLeft, *mpLineextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;
    int N_l;
    int N_p;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;
    // Vector of keylines
    std::vector<cv::line_descriptor::KeyLine> mvKeys_Line, mvKeysRight_Line;
    std::vector<cv::line_descriptor::KeyLine> mvKeysUn_Line;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // for PLslam
    // std::vector<LineFeature*> stereo_ls;
    std::vector<pair<float,float>> mvDisparity_l;
    std::vector<Vector3d> mvle_l;

    // Flag to identify outlier associations.
    std::vector<Vector3d> mv3DpointInPrevFrame;
    std::vector<pair<Vector3d,Vector3d>> mv3DlineInPrevFrame;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;
    // bag of word pairs
    map<WordId,list<WordId>> mwordPairs;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec_l;
    DBoW2::FeatureVector mFeatVec_l;


    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;
    // Line descriptor, each row associated to a keyline.
    cv::Mat mDescriptors_Line, mDescriptorsRight_Line;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;
    // MapLines associated to keylines, NULL pointer if no association.
    std::vector<MapLine*> mvpMapLines;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    std::vector<bool> mvbOutlier_Line;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;
    cv::Mat mTcw_prev;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    // grid cell
    double inv_width, inv_height; 

//    Matrix6d Twf_cov;
//    Vector6d Twf_cov_eig;       //T^w_f

    Matrix4d DT;
    Matrix6d DT_cov;
    Vector6d DT_cov_eig;
    double   err_norm;

    int  n_inliers, n_inliers_pt, n_inliers_ls;

private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();
    // Undistort keylines
    void UndistortKeyLines();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // PLslam
    double lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj);
    void filterLineSegmentDisparity(Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e);

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
