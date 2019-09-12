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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpLinevocabulary(frame.mpLinevocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mpLineextractorLeft(frame.mpLineextractorLeft), mpLineextractorRight(frame.mpLineextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), N_l(frame.N_l),
     mvKeys(frame.mvKeys), mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),
     mvKeys_Line(frame.mvKeys_Line), mvKeysRight_Line(frame.mvKeysRight_Line), mvKeysUn_Line(frame.mvKeysUn_Line),
     mvuRight(frame.mvuRight), mvDepth(frame.mvDepth), mvDisparity_l(frame.mvDisparity_l), mvle_l(frame.mvle_l),
     mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),mBowVec_l(frame.mBowVec_l), mFeatVec_l(frame.mFeatVec_l),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mDescriptors_Line(frame.mDescriptors_Line.clone()), mDescriptorsRight_Line(frame.mDescriptorsRight_Line.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvpMapLines(frame.mvpMapLines),
     mvbOutlier(frame.mvbOutlier), mvbOutlier_Line(frame.mvbOutlier_Line), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), inv_width(frame.inv_width), inv_height(frame.inv_width),
     DT(frame.DT), DT_cov(frame.DT_cov), err_norm(frame.err_norm),
     n_inliers(frame.n_inliers), n_inliers_pt(frame.n_inliers_pt), n_inliers_ls(frame.n_inliers_ls)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty()) {
        SetPose(frame.mTcw);
        mTcw_prev = frame.mTcw_prev.clone();
    }
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, 
    ORBextractor* extractorLeft, ORBextractor* extractorRight, 
    ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    if (imLeft.size != imRight.size)
        throw std::runtime_error("[StereoFrame] Left and right images have different sizes");

    inv_width  = FRAME_GRID_COLS / static_cast<double>(imLeft.cols);
    inv_height = FRAME_GRID_ROWS / static_cast<double>(imRight.rows);

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction && LineFeatures extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

// Constructor for stereo cameras with lines
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, 
    ORBextractor* extractorLeft, ORBextractor* extractorRight, 
    Lineextractor* LineextractorLeft, Lineextractor* LineextractorRight,
    ORBVocabulary* voc, LineVocabulary* voc_l, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc), mpLinevocabulary(voc_l), mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight),
     mpLineextractorLeft(LineextractorLeft),mpLineextractorRight(LineextractorRight),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    if (imLeft.size != imRight.size)
        throw std::runtime_error("[StereoFrame] Left and right images have different sizes");

    inv_width  = FRAME_GRID_COLS / static_cast<double>(imLeft.cols);
    inv_height = FRAME_GRID_ROWS / static_cast<double>(imRight.rows);

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction && LineFeatures extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    thread threadLeft_Line(&Frame::ExtractLine,this,0,imLeft);
    thread threadRight_Line(&Frame::ExtractLine,this,1,imRight);
    threadLeft.join();
    threadRight.join();
    threadLeft_Line.join();
    threadRight_Line.join();

    N = mvKeys.size();
    N_l = mvKeys_Line.size();       // for now

    if(mvKeys.empty())
        return;

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    UndistortKeyPoints();
    //UndistortKeyLines();

    ComputeStereoMatches();
    if(Config::hasLines())
    {
        ComputeStereoMatches_Lines();           //use mvKeys_Line
        N_l = mvKeys_Line.size();   // update N_l
        UndistortKeyLines();
    }

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvpMapLines = vector<MapLine*>(N_l,static_cast<MapLine*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mvbOutlier_Line = vector<bool>(N_l,false);

    AssignFeaturesToGrid();

    DT = Matrix4d::Identity();
    DT_cov = Matrix6d::Zero();
    DT_cov_eig = Vector6d::Zero();
    err_norm = -1.0;
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::ExtractLine(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpLineextractorLeft)(im,cv::Mat(),mvKeys_Line,mDescriptors_Line);
    else
        (*mpLineextractorRight)(im,cv::Mat(),mvKeysRight_Line,mDescriptorsRight_Line);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

void Frame::SetprevInformation(cv::Mat _Tcw, Matrix6d _DT_cov, double _err_norm)
{
    mTcw_prev = _Tcw.clone();
    DT_cov = _DT_cov;
    err_norm = _err_norm;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

bool Frame::isInFrustum_l(MapLine *pML, float viewingCosLimit)
{
    pML->mbTrackInView = false;

    // 3D in absolute coordinates
    Vector6d sep = pML->GetWorldPos();
    Vector3d sp_eigen = sep.head(3);
    Vector3d ep_eigen = sep.tail(3);
    cv::Mat sp = Converter::toCvMat(sp_eigen);
    cv::Mat ep = Converter::toCvMat(ep_eigen);
    {
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*sp+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        pML->mTrackProjsX = u;
        pML->mTrackProjsY = v;
    }
    {
        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*ep+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        pML->mTrackProjeX = u;
        pML->mTrackProjeY = v;
    }

    // TODO: check distance+viewing angle, predict scale
    // Check distance is in the scale invariance region of the MapPoint
    // Check viewing angle
    // Predict scale in the image

    // Data used by the tracking
    pML->mbTrackInView = true;
    pML->mnTrackangle = atan2(pML->mTrackProjeY - pML->mTrackProjsY, pML->mTrackProjeX - pML->mTrackProjsX);

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
    if(mBowVec_l.empty())
    {
        vector<cv::Mat> vCurrentDesc_l = Converter::toDescriptorVector(mDescriptors_Line);
        mpLinevocabulary->transform(vCurrentDesc_l,mBowVec_l,mFeatVec_l,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::UndistortKeyLines()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn_Line = mvKeys_Line;
        return;
    }

    N_l = mvKeys_Line.size();   // update N_l

    // Fill matrix with points
    cv::Mat mat_s(N_l,2,CV_32F);
    cv::Mat mat_e(N_l,2,CV_32F);
    for(int i=0; i<N_l; i++)
    {
        mat_s.at<float>(i,0)=mvKeys_Line[i].startPointX;
        mat_s.at<float>(i,1)=mvKeys_Line[i].startPointY;
        mat_e.at<float>(i,0)=mvKeys_Line[i].endPointX;
        mat_e.at<float>(i,1)=mvKeys_Line[i].endPointY;
    }

    // Undistort points
    mat_s=mat_s.reshape(2);
    mat_e=mat_e.reshape(2);
    cv::undistortPoints(mat_s,mat_s,mK,mDistCoef,cv::Mat(),mK);
    cv::undistortPoints(mat_e,mat_e,mK,mDistCoef,cv::Mat(),mK);
    mat_s=mat_s.reshape(1);
    mat_e=mat_e.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn_Line.resize(N_l);
    for(int i=0; i<N_l; i++)
    {
        mvKeysUn_Line[i].startPointX=mat_s.at<float>(i,0);
        mvKeysUn_Line[i].startPointY=mat_s.at<float>(i,1);
        mvKeysUn_Line[i].endPointX=mat_e.at<float>(i,0);
        mvKeysUn_Line[i].endPointY=mat_e.at<float>(i,1);
    }

}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

void Frame::ComputeStereoMatches_Lines(bool initial)
{
    bool doNotDropMonoLines = true;

    std::vector<cv::line_descriptor::KeyLine> mvKeys_Line_tmp;
    mvKeys_Line_tmp.reserve(mvKeys_Line.size());
    mvDisparity_l.clear();
    mvle_l.clear();
    if(doNotDropMonoLines){
        mvDisparity_l.resize(mvKeys_Line.size(),pair<float,float>(-1,-1));
        mvle_l.resize(mvKeys_Line.size(),Vector3d(0,0,0));
    }
    else {
        mvDisparity_l.reserve(mvKeys_Line.size());
        mvle_l.reserve(mvKeys_Line.size());
    }

    // Line segments stereo matching
    // --------------------------------------------------------------------------------------------------------------------
    if (mvKeys_Line.empty() || mvKeysRight_Line.empty())
        return;

    std::vector<line_2d> coords;
    coords.reserve(mvKeys_Line.size());
    for (const KeyLine &kl : mvKeys_Line)
        coords.push_back(std::make_pair(std::make_pair(kl.startPointX * inv_width, kl.startPointY * inv_height),
                                        std::make_pair(kl.endPointX * inv_width, kl.endPointY * inv_height)));

    //Fill in grid & directions
    list<pair<int, int>> line_coords;
    GridStructure grid(FRAME_GRID_ROWS, FRAME_GRID_COLS);
    std::vector<std::pair<double, double>> directions(mvKeysRight_Line.size());
    for (unsigned int idx = 0; idx < mvKeysRight_Line.size(); ++idx) {
        const KeyLine &kl = mvKeysRight_Line[idx];

        std::pair<double, double> &v = directions[idx];
        v = std::make_pair((kl.endPointX - kl.startPointX) * inv_width, (kl.endPointY - kl.startPointY) * inv_height);
        normalize(v);

        getLineCoords(kl.startPointX * inv_width, kl.startPointY * inv_height, kl.endPointX * inv_width, kl.endPointY * inv_height, line_coords);
        for (const std::pair<int, int> &p : line_coords)
            grid.at(p.first, p.second).push_back(idx);
    }

    GridWindow w;
    w.width = std::make_pair(Config::matchingSWs(), 0);
    w.height = std::make_pair(0, 0);

    std::vector<int> matches_12;
    matchGrid(coords, mDescriptors_Line, grid, mDescriptorsRight_Line, directions, w, matches_12);
    // match(mDescriptors_Line, mDescriptorsRight_Line, Config::minRatio12P(), matches_12);

    // bucle around lmatches
    Mat mDescriptors_Line_aux;
    // int ls_idx = 0;
    // stereo_ls.clear();
    for (unsigned int i1 = 0; i1 < matches_12.size(); ++i1) {
        const int i2 = matches_12[i1];
        if (i2 < 0) continue;

        // estimate the disparity of the endpoints
        Vector3d sp_l; sp_l << mvKeys_Line[i1].startPointX, mvKeys_Line[i1].startPointY, 1.0;
        Vector3d ep_l; ep_l << mvKeys_Line[i1].endPointX,   mvKeys_Line[i1].endPointY,   1.0;
        Vector3d le_l; le_l << sp_l.cross(ep_l); le_l = le_l / std::sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        Vector3d sp_r; sp_r << mvKeysRight_Line[i2].startPointX, mvKeysRight_Line[i2].startPointY, 1.0;
        Vector3d ep_r; ep_r << mvKeysRight_Line[i2].endPointX,   mvKeysRight_Line[i2].endPointY,   1.0;
        Vector3d le_r; le_r << sp_r.cross(ep_r);

        double overlap = lineSegmentOverlapStereo( sp_l(1), ep_l(1), sp_r(1), ep_r(1) );

        double disp_s, disp_e;
        sp_r << ( sp_r(0)*( sp_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - sp_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , sp_l(1) ,  1.0;
        ep_r << ( sp_r(0)*( ep_l(1) - ep_r(1) ) + ep_r(0)*( sp_r(1) - ep_l(1) ) ) / ( sp_r(1)-ep_r(1) ) , ep_l(1) ,  1.0;
        filterLineSegmentDisparity( sp_l.head(2), ep_l.head(2), sp_r.head(2), ep_r.head(2), disp_s, disp_e );

        // check minimal disparity
        if( disp_s >= Config::minDisp() && disp_e >= Config::minDisp()
            && std::abs( sp_l(1)-ep_l(1) ) > Config::lineHorizTh()
            && std::abs( sp_r(1)-ep_r(1) ) > Config::lineHorizTh()
            && overlap > Config::stereoOverlapTh() )
        {
            // TODO : check if works well
            if(doNotDropMonoLines) {
                mvDisparity_l[i1] = make_pair(disp_s,disp_e);
                mvle_l[i1] = le_l;
            }
            else {
                mvKeys_Line_tmp.push_back(mvKeys_Line[i1]);
                mvDisparity_l.push_back(make_pair(disp_s,disp_e));
                mvle_l.push_back(le_l);
                mDescriptors_Line_aux.push_back( mDescriptors_Line.row(i1) );
            }

            // use mvKeys_Line+mvDisparity_l+mvle_l instead of stereo_ls
            /*
            Vector3d sP_; //sP_ = backProjection( sp_l(0), sp_l(1), disp_s);
            Vector3d eP_; //eP_ = backProjection( ep_l(0), ep_l(1), disp_e);
            double angle_l = mvKeys_Line[i1].angle;
            if( initial )
            {
                mDescriptors_Line_aux.push_back( mDescriptors_Line.row(i1) );
                stereo_ls.push_back( new LineFeature(Vector2d(sp_l(0),sp_l(1)),disp_s,sP_,
                                                     Vector2d(ep_l(0),ep_l(1)),disp_e,eP_,
                                                     le_l,angle_l,ls_idx,mvKeys_Line[i1].octave) );
                ls_idx++;
            }
            else
            {
                mDescriptors_Line_aux.push_back( mDescriptors_Line.row(i1) );
                stereo_ls.push_back( new LineFeature(Vector2d(sp_l(0),sp_l(1)),disp_s,sP_,
                                                     Vector2d(ep_l(0),ep_l(1)),disp_e,eP_,
                                                     le_l,angle_l,-1,mvKeys_Line[i1].octave) );
            }
            */
        }
    }
    if(doNotDropMonoLines)
        ;
    else {
        mvKeys_Line.swap(mvKeys_Line_tmp);
        mDescriptors_Line_aux.copyTo(mDescriptors_Line);
    }
}

double Frame::lineSegmentOverlapStereo(double spl_obs, double epl_obs, double spl_proj, double epl_proj)
{

    double overlap = 1.f;

    if( fabs( epl_obs - spl_obs ) > Config::lineHorizTh() ) // normal lines (verticals included)
    {
        double sln    = min(spl_obs,  epl_obs);
        double eln    = max(spl_obs,  epl_obs);
        double spn    = min(spl_proj, epl_proj);
        double epn    = max(spl_proj, epl_proj);

        double length = eln-spn;

        if ( (epn < sln) || (spn > eln) )
            overlap = 0.f;
        else{
            if ( (epn>eln) && (spn<sln) )
                overlap = eln-sln;
            else
                overlap = min(eln,epn) - max(sln,spn);
        }

        if(length>0.01f)
            overlap = overlap / length;
        else
            overlap = 0.f;

        if( overlap > 1.f )
            overlap = 1.f;

    }

    return overlap;
}

void Frame::filterLineSegmentDisparity( Vector2d spl, Vector2d epl, Vector2d spr, Vector2d epr, double &disp_s, double &disp_e )
{
    disp_s = spl(0) - spr(0);
    disp_e = epl(0) - epr(0);
    // if they are too different, ignore them
    if(  min( disp_s, disp_e ) / max( disp_s, disp_e ) < Config::lsMinDispRatio() )
    {
        disp_s = -1.0;
        disp_e = -1.0;
    }
}

void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

Eigen::Vector3d Frame::backProjection( const double &u, const double &v, const double &disp )
{
    // convert the point in image to the point in world coordinate.
    Eigen::Vector3d P;      // point in camera coordinate
    double bd = mb/disp;
    P(0) = bd*(u-cx);
    P(1) = bd*(v-cy);
    P(2) = bd*fx;
    return Converter::toMatrix3d(mRwc)*P+Converter::toVector3d(mOw);
}
Eigen::Vector2d Frame::projection(const Eigen::Vector3d &P )
{
    // convert the 3Dpoint in camera coordinate to the point in image.
    Vector2d uv_unit;
    uv_unit(0) = cx + fx * P(0) / P(2);
    uv_unit(1) = cy + fy * P(1) / P(2);
    return uv_unit;
}

double Frame::lineSegmentOverlap(Vector2d spl_obs, Vector2d epl_obs, Vector2d spl_proj, Vector2d epl_proj)
{

    double overlap = 1.f;

    if( fabs(spl_obs(0)-epl_obs(0)) < 1.0 )         // vertical lines
    {

        // line equations
        Vector2d l = epl_obs - spl_obs;

        // intersection points
        Vector2d spl_proj_line, epl_proj_line;
        spl_proj_line << spl_obs(0), spl_proj(1);
        epl_proj_line << epl_obs(0), epl_proj(1);

        // estimate overlap in function of lambdas
        double lambda_s = (spl_proj_line(1)-spl_obs(1)) / l(1);
        double lambda_e = (epl_proj_line(1)-spl_obs(1)) / l(1);

        double lambda_min = min(lambda_s,lambda_e);
        double lambda_max = max(lambda_s,lambda_e);

        if( lambda_min < 0.f && lambda_max > 1.f )
            overlap = 1.f;
        else if( lambda_max < 0.f || lambda_min > 1.f )
            overlap = 0.f;
        else if( lambda_min < 0.f )
            overlap = lambda_max;
        else if( lambda_max > 1.f )
            overlap = 1.f - lambda_min;
        else
            overlap = lambda_max - lambda_min;
    }
    else if( fabs(spl_obs(1)-epl_obs(1)) < 1.0 )    // horizontal lines (previously removed)
    {

        // line equations
        Vector2d l = epl_obs - spl_obs;

        // intersection points
        Vector2d spl_proj_line, epl_proj_line;
        spl_proj_line << spl_proj(0), spl_obs(1);
        epl_proj_line << epl_proj(0), epl_obs(1);

        // estimate overlap in function of lambdas
        double lambda_s = (spl_proj_line(0)-spl_obs(0)) / l(0);
        double lambda_e = (epl_proj_line(0)-spl_obs(0)) / l(0);

        double lambda_min = min(lambda_s,lambda_e);
        double lambda_max = max(lambda_s,lambda_e);

        if( lambda_min < 0.f && lambda_max > 1.f )
            overlap = 1.f;
        else if( lambda_max < 0.f || lambda_min > 1.f )
            overlap = 0.f;
        else if( lambda_min < 0.f )
            overlap = lambda_max;
        else if( lambda_max > 1.f )
            overlap = 1.f - lambda_min;
        else
            overlap = lambda_max - lambda_min;
    }
    else                                            // non-degenerate cases
    {

        // line equations
        Vector2d l = epl_obs - spl_obs;
        double a = spl_obs(1)-epl_obs(1);
        double b = epl_obs(0)-spl_obs(0);
        double c = spl_obs(0)*epl_obs(1) - epl_obs(0)*spl_obs(1);

        // intersection points
        Vector2d spl_proj_line, epl_proj_line;
        double lxy = 1.f / (a*a+b*b);

        spl_proj_line << ( b*( b*spl_proj(0)-a*spl_proj(1))-a*c ) * lxy,
                         ( a*(-b*spl_proj(0)+a*spl_proj(1))-b*c ) * lxy;

        epl_proj_line << ( b*( b*epl_proj(0)-a*epl_proj(1))-a*c ) * lxy,
                         ( a*(-b*epl_proj(0)+a*epl_proj(1))-b*c ) * lxy;

        // estimate overlap in function of lambdas
        double lambda_s = (spl_proj_line(0)-spl_obs(0)) / l(0);
        double lambda_e = (epl_proj_line(0)-spl_obs(0)) / l(0);

        double lambda_min = min(lambda_s,lambda_e);
        double lambda_max = max(lambda_s,lambda_e);

        if( lambda_min < 0.f && lambda_max > 1.f )
            overlap = 1.f;
        else if( lambda_max < 0.f || lambda_min > 1.f )
            overlap = 0.f;
        else if( lambda_min < 0.f )
            overlap = lambda_max;
        else if( lambda_max > 1.f )
            overlap = 1.f - lambda_min;
        else
            overlap = lambda_max - lambda_min;
    }
    return overlap;
}

} //namespace ORB_SLAM
