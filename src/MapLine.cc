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

#include "MapLine.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapLine::nNextId=0;

mutex MapLine::mGlobalMutex;

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, Map* pMap):
    mnFirstKFid(-1), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0),
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    // mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
}

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP, KeyFrame* pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0),
    mpRefKF(pRefKF), mnVisible(1), mnFound(1),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    // mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

MapLine::MapLine(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP,  Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0),mnLastFrameSeen(0),
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1), mnFound(1),
    mbBad(false), mpReplaced(static_cast<MapLine*>(NULL)), mpMap(pMap)
{
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
    // cv::Mat Ow = pFrame->GetCameraCenter();
    // mNormalVector = mWorldPos - Ow;
    // mNormalVector = mNormalVector/cv::norm(mNormalVector);

    // cv::Mat PC = Pos - Ow;
    // const float dist = cv::norm(PC);
    // const int level = pFrame->mvKeysUn[idxF].octave;
    // const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    // const int nLevels = pFrame->mnScaleLevels;

    // mfMaxDistance = dist*levelScaleFactor;
    // mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors_Line.row(idxF).copyTo(mDescriptor);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

void MapLine::SetWorldPos(const Eigen::Vector3d &sP, const Eigen::Vector3d &eP)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPos_sP = sP;
    mWorldPos_eP = eP;
}

Vector6d MapLine::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    Vector6d sep;
    sep.head(3) = mWorldPos_sP;
    sep.tail(3) = mWorldPos_eP;
    return sep;
}


KeyFrame* MapLine::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapLine::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    nObs+=2;
    // if(pKF->mvuRight[idx]>=0)
    //     nObs+=2;
    // else
    //     nObs++;
}

void MapLine::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            // int idx = mObservations[pKF];
            nObs-=2;
            // if(pKF->mvuRight[idx]>=0)
            //     nObs-=2;
            // else
            //     nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapLine::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapLine::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapLine::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapLineMatch(mit->second);
    }

    mpMap->EraseMapLine(this);
}

MapLine* MapLine::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapLine::Replace(MapLine* pML)
{
    if(pML->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pML;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pML->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapLineMatch(mit->second, pML);
            pML->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapLineMatch(mit->second);
        }
    }
    pML->IncreaseFound(nfound);
    pML->IncreaseVisible(nvisible);
    pML->ComputeDistinctiveDescriptors();

    mpMap->EraseMapLine(this);
}

bool MapLine::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapLine::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapLine::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapLine::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapLine::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors_l.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapLine::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

void MapLine::UpdateNormalAndDepth()
{
    // TODO: MapLine::UpdateNormalAndDepth()

    // map<KeyFrame*,size_t> observations;
    // KeyFrame* pRefKF;
    // cv::Mat Pos;
    // {
    //     unique_lock<mutex> lock1(mMutexFeatures);
    //     unique_lock<mutex> lock2(mMutexPos);
    //     if(mbBad)
    //         return;
    //     observations=mObservations;
    //     pRefKF=mpRefKF;
    //     Pos = mWorldPos.clone();
    // }

    // if(observations.empty())
    //     return;

    // cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    // int n=0;
    // for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    // {
    //     KeyFrame* pKF = mit->first;
    //     cv::Mat Owi = pKF->GetCameraCenter();
    //     cv::Mat normali = mWorldPos - Owi;
    //     normal = normal + normali/cv::norm(normali);
    //     n++;
    // }

    // cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    // const float dist = cv::norm(PC);
    // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    // const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    // const int nLevels = pRefKF->mnScaleLevels;

    // {
    //     unique_lock<mutex> lock3(mMutexPos);
    //     mfMaxDistance = dist*levelScaleFactor;
    //     mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
    //     mNormalVector = normal/n;
    // }
}

int MapLine::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapLine::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

KeyFrame* MapLine::SetReferenceKeyFrame(KeyFrame* RFKF)
{
    return mpRefKF = RFKF;
}

} //namespace ORB_SLAM
