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

#ifndef MAPLINE_H
#define MAPLINE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>
#include <map>

#include <eigen3/Eigen/Core>
using namespace Eigen;

typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

class MapLine
{
public:
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, Map* pMap);
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, KeyFrame* pRefKF, Map* pMap);
    MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP);
    Vector6d GetWorldPos();

    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapLine* pML);    
    MapLine* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }
    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    KeyFrame* SetReferenceKeyFrame(KeyFrame* RFKF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    static std::mutex mGlobalMutex;

     // Position in absolute coordinates
     Eigen::Vector3d mWorldPos_sP;
     Eigen::Vector3d mWorldPos_eP;

     // Tracking
     bool mbTrackInView;
     float mTrackProjsX;
     float mTrackProjsY;
     float mTrackProjeX;
     float mTrackProjeY;
     double mnTrackangle;
     long unsigned int mnTrackReferenceForFrame;
     long unsigned int mnLastFrameSeen;

protected:    

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapLine* mpReplaced;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPLINE_H
