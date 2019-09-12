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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::AddMapLine(MapLine* pML)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLines.insert(pML); 
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseMapLine(MapLine *pML)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLines.erase(pML);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::SetReferenceMapLines(const vector<MapLine *> &vpMLs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapLines = vpMLs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapLine*> Map::GetAllMapLines()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapLine*>(mspMapLines.begin(),mspMapLines.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::MapLinesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapLines.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

vector<MapLine*> Map::GetReferenceMapLines()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapLines;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<MapLine*>::iterator sit=mspMapLines.begin(), send=mspMapLines.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspMapLines.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpReferenceMapLines.clear();
    mvpKeyFrameOrigins.clear();
}

long unsigned int Map::GetMaxMPid()
{
    unique_lock<mutex> lock(mMutexMap);
    long unsigned int MaxMPid = 0;
    for(auto iter = mspMapPoints.begin(); iter != mspMapPoints.end(); ++iter)
    {
        if( (*iter) -> mnId > MaxMPid)
            MaxMPid = (*iter)->mnId;
    }
    return MaxMPid;
}

long unsigned int Map::GetMaxFid()
{
    unique_lock<mutex> lock(mMutexMap);
    long unsigned int MaxFid = 0;
    for(auto iter = mspKeyFrames.begin(); iter != mspKeyFrames.end(); ++iter)
    {
        if( (*iter) -> mnId > MaxFid)
            MaxFid = (*iter)->mnId;
    }
    return MaxFid;
}

void Map::SaveMapPoint(ofstream& f, MapPoint* mp)
{
	f.write((char*)&mp->mnId, sizeof(mp->mnId));
	f.write((char*)&mp->mnFirstKFid, sizeof(mp->mnFirstKFid));
	f.write((char*)&mp->mnFirstFrame, sizeof(mp->mnFirstFrame));
	f.write((char*)&mp->nObs, sizeof(mp->nObs));

    cv::Mat mpWorldPos = mp->GetWorldPos();
    f.write((char*)& mpWorldPos.at<float>(0), sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(1), sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(2), sizeof(float));
}

MapPoint* Map::LoadMapPoint(ifstream& f)
{
    long unsigned int id;
    long int nFirstKFid, nFirstFrame;
    int obs;
    f.read((char*)&id, sizeof(long unsigned int));
    f.read((char*)&nFirstKFid, sizeof(long int));
    f.read((char*)&nFirstFrame, sizeof(long int));
    f.read((char*)&obs, sizeof(int));

    cv::Mat Position(3,1,CV_32F);
    f.read((char*)&Position.at<float>(0), sizeof(float));
    f.read((char*)&Position.at<float>(1), sizeof(float));
    f.read((char*)&Position.at<float>(2), sizeof(float));

    //init a mappoint and set id & position
    MapPoint *mp = new MapPoint(Position, this);
    mp->mnId = id;
    mp->mnFirstKFid = nFirstKFid;
    mp->mnFirstFrame = nFirstFrame;
    mp->nObs = obs;

    return mp;
}

void Map::SaveMapLine(ofstream& f, MapLine* ml)
{
	f.write((char*)&ml->mnId, sizeof(ml->mnId));
	f.write((char*)&ml->mnFirstKFid, sizeof(ml->mnFirstKFid));
	f.write((char*)&ml->mnFirstFrame, sizeof(ml->mnFirstFrame));
	f.write((char*)&ml->nObs, sizeof(ml->nObs));

    Vector6d mpWorldPos = ml->GetWorldPos();
    f.write((char*)& mpWorldPos(0), sizeof(double));
    f.write((char*)& mpWorldPos(1), sizeof(double));
    f.write((char*)& mpWorldPos(2), sizeof(double));
    f.write((char*)& mpWorldPos(3), sizeof(double));
    f.write((char*)& mpWorldPos(4), sizeof(double));
    f.write((char*)& mpWorldPos(5), sizeof(double));
}

MapLine* Map::LoadMapLine(ifstream& f)
{
    long unsigned int id;
    long int nFirstKFid, nFirstFrame;
    int obs;
    f.read((char*)&id, sizeof(long unsigned int));
    f.read((char*)&nFirstKFid, sizeof(long int));
    f.read((char*)&nFirstFrame, sizeof(long int));
    f.read((char*)&obs, sizeof(int));

    Vector3d sp, ep;
    f.read((char*)&sp(0), sizeof(double));
    f.read((char*)&sp(1), sizeof(double));
    f.read((char*)&sp(2), sizeof(double));
    f.read((char*)&ep(0), sizeof(double));
    f.read((char*)&ep(1), sizeof(double));
    f.read((char*)&ep(2), sizeof(double));

    //init a mapline and set id & position
    MapLine *ml = new MapLine(sp, ep, this);
    ml->mnId = id;
    ml->mnFirstKFid = nFirstKFid;
    ml->mnFirstFrame = nFirstFrame;
    ml->nObs = obs;

    return ml;
}

void Map::SaveKeyFrame(ofstream& f, KeyFrame* kf)
{
    f.write((char*)&kf->mnFrameId, sizeof(kf->mnFrameId));
    f.write((char*)&kf->mnId, sizeof(kf->mnId));
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));

    cv::Mat Tcw = kf->GetPose();
    std::vector<float> Quat = Converter::toQuaternion(Tcw);

    for(int i = 0; i < 3; ++i)
        f.write((char*)&Tcw.at<float>(i,3), sizeof(float));

    for(int i = 0; i < 4; i++)
        f.write((char*)&Quat[i], sizeof(float));

    // write KeyPoints&MapPoints
    f.write((char*)&kf->N, sizeof(kf->N));
    for(int i = 0; i < kf->N; i++)
    {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.write((char*)&kp.size, sizeof(kp.size));
        f.write((char*)&kp.angle, sizeof(kp.angle));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.octave, sizeof(kp.octave));

        float uRight = kf->mvuRight[i];
        f.write((char*)&uRight, sizeof(uRight));
        float depth = kf->mvDepth[i];
        f.write((char*)&depth, sizeof(depth));

        for(int j = 0;j < kf->mDescriptors.cols; ++j)
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j),sizeof(char));

        unsigned long int mnIdx;
        MapPoint* mp = kf->GetMapPoint(i);
        if(mp == NULL)
            mnIdx = ULONG_MAX;
        else
            mnIdx = mp->mnId;

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }

#ifdef HasLine
    // write KeyLines&MapLines
    f.write((char*)&kf->N_l, sizeof(kf->N_l));
    for(int i = 0; i < kf->N_l; i++)
    {
        cv::line_descriptor::KeyLine kl = kf->mvKeys_Line[i];
        f.write((char*)&kl.angle, sizeof(kl.angle));
        f.write((char*)&kl.class_id, sizeof(kl.class_id));
        f.write((char*)&kl.octave, sizeof(kl.octave));
        f.write((char*)&kl.pt.x, sizeof(kl.pt.x));
        f.write((char*)&kl.pt.y, sizeof(kl.pt.y));
        f.write((char*)&kl.response, sizeof(kl.response));
        f.write((char*)&kl.size, sizeof(kl.size));
        f.write((char*)&kl.startPointX, sizeof(kl.startPointX));
        f.write((char*)&kl.startPointY, sizeof(kl.startPointY));
        f.write((char*)&kl.endPointX, sizeof(kl.endPointX));
        f.write((char*)&kl.endPointY, sizeof(kl.endPointY));
        f.write((char*)&kl.sPointInOctaveX, sizeof(kl.sPointInOctaveX));
        f.write((char*)&kl.sPointInOctaveY, sizeof(kl.sPointInOctaveY));
        f.write((char*)&kl.ePointInOctaveX, sizeof(kl.ePointInOctaveX));
        f.write((char*)&kl.ePointInOctaveY, sizeof(kl.ePointInOctaveY));
        f.write((char*)&kl.lineLength, sizeof(kl.lineLength));
        f.write((char*)&kl.numOfPixels, sizeof(kl.numOfPixels));

        pair<float,float> disparity_l = kf->mvDisparity_l[i];
        f.write((char*)&disparity_l.first, sizeof(disparity_l.first));
        f.write((char*)&disparity_l.second, sizeof(disparity_l.second));

        Vector3d le_l = kf->mvle_l[i];
        f.write((char*)&le_l(0), sizeof(double));
        f.write((char*)&le_l(1), sizeof(double));
        f.write((char*)&le_l(2), sizeof(double));

        for(int j = 0;j < kf->mDescriptors_l.cols; ++j)
            f.write((char*)&kf->mDescriptors_l.at<unsigned char>(i,j),sizeof(char));

        unsigned long int mnIdx;
        MapLine* ml = kf->GetMapLine(i);
        if(ml == NULL)
            mnIdx = ULONG_MAX;
        else
            mnIdx = ml->mnId;

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }
#endif
}

KeyFrame* Map::LoadKeyFrame(ifstream& f, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB)
{
    //init keyframe
    InitKeyFrame initkf(*mySystemSetting);
    //read keyframe id & timestamp
    f.read((char*)&initkf.mnFrameId, sizeof(initkf.mnFrameId));
    f.read((char*)&initkf.nId, sizeof(initkf.nId));
    f.read((char*)&initkf.TimeStamp, sizeof(double));

    //read keyframe pos&ori
    cv::Mat Tcw(4,4,CV_32F);
    for( int i = 0; i < 4; ++i)
        for( int j = 0; j < 4; ++j)
            Tcw.at<float>(i,j) = 0;

    for( int i = 0; i < 3; ++i)
       f.read((char*)&Tcw.at<float>(i,3), sizeof(float));
    Tcw.at<float>(3,3) = 1;
    cv::Mat Qcw(1, 4, CV_32F);
    for( int i = 0; i < 4; ++i)
       f.read((char*)&Qcw.at<float>(0,i), sizeof(float));

    Converter::RmatOfQuat(Tcw, Qcw);

    std::vector<MapPoint*> vpMapPoints;
    std::vector<MapLine*> vpMapLines;

    f.read((char*)&initkf.N, sizeof(initkf.N));
    vpMapPoints = vector<MapPoint*>(initkf.N, static_cast<MapPoint*>(NULL));
    initkf.vKps.reserve(initkf.N);
    initkf.Descriptors.create(initkf.N, 32, CV_8UC1);

    //read this keyframe keypoint & descriptor
    for(int i = 0; i < initkf.N; ++i )
    {
        cv::KeyPoint kp;
        f.read((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.read((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.read((char*)&kp.size, sizeof(kp.size));
        f.read((char*)&kp.angle, sizeof(kp.angle));
        f.read((char*)&kp.response, sizeof(kp.response));
        f.read((char*)&kp.octave, sizeof(kp.octave));
        initkf.vKps.push_back(kp);

        float uRight;
        f.read((char*)&uRight, sizeof(uRight));
        initkf.vRight.push_back(uRight);

        float depth;
        f.read((char*)&depth, sizeof(depth));
        initkf.vDepth.push_back(depth);

        for( int j = 0; j < 32; ++j)
            f.read((char*)&initkf.Descriptors.at<unsigned char>(i,j), sizeof(char));

        //read this keypoint's relationship with mappoint
        unsigned long int mpidx;
        f.read((char*)&mpidx, sizeof(mpidx));

        //read all mappoints & find this keyframe mappoint
        if( mpidx == ULONG_MAX )
            vpMapPoints[i] = NULL;
        else
            vpMapPoints[i] = mmpnIdx2MapPoints[mpidx];
    }

    initkf.UndistortKeyPoints();
    initkf.AssignFeaturesToGrid();

#ifdef HasLine
    //read keyline number
    f.read((char*)&initkf.N_l, sizeof(initkf.N_l));
    vpMapLines = vector<MapLine*>(initkf.N_l, static_cast<MapLine*>(NULL));
    initkf.vKeys_Line.reserve(initkf.N_l);
    initkf.Descriptors_Line.create(initkf.N_l, 32, CV_8UC1);

    //read this keyframe keypoint & descriptor
    for(int i = 0; i < initkf.N_l; ++i )
    {
        cv::line_descriptor::KeyLine kl;
        f.read((char*)&kl.angle, sizeof(kl.angle));
        f.read((char*)&kl.class_id, sizeof(kl.class_id));
        f.read((char*)&kl.octave, sizeof(kl.octave));
        f.read((char*)&kl.pt.x, sizeof(kl.pt.x));
        f.read((char*)&kl.pt.y, sizeof(kl.pt.y));
        f.read((char*)&kl.response, sizeof(kl.response));
        f.read((char*)&kl.size, sizeof(kl.size));
        f.read((char*)&kl.startPointX, sizeof(kl.startPointX));
        f.read((char*)&kl.startPointY, sizeof(kl.startPointY));
        f.read((char*)&kl.endPointX, sizeof(kl.endPointX));
        f.read((char*)&kl.endPointY, sizeof(kl.endPointY));
        f.read((char*)&kl.sPointInOctaveX, sizeof(kl.sPointInOctaveX));
        f.read((char*)&kl.sPointInOctaveY, sizeof(kl.sPointInOctaveY));
        f.read((char*)&kl.ePointInOctaveX, sizeof(kl.ePointInOctaveX));
        f.read((char*)&kl.ePointInOctaveY, sizeof(kl.ePointInOctaveY));
        f.read((char*)&kl.lineLength, sizeof(kl.lineLength));
        f.read((char*)&kl.numOfPixels, sizeof(kl.numOfPixels));        
        initkf.vKeys_Line.push_back(kl);

        pair<float,float> disparity_l;
        f.read((char*)&disparity_l.first, sizeof(disparity_l.first));
        f.read((char*)&disparity_l.second, sizeof(disparity_l.second));
        initkf.vDisparity_l.push_back(disparity_l);

        Vector3d le_l;
        f.read((char*)&le_l(0), sizeof(double));
        f.read((char*)&le_l(1), sizeof(double));
        f.read((char*)&le_l(2), sizeof(double));
        initkf.vle_l.push_back(le_l);

        for( int j = 0; j < 32; ++j)
            f.read((char*)&initkf.Descriptors_Line.at<unsigned char>(i,j), sizeof(char));

        //read this keypoint's relationship with mappoint
        unsigned long int mpidx;
        f.read((char*)&mpidx, sizeof(mpidx));

        //read all mappoints & find this keyframe mappoint
        if( mpidx == ULONG_MAX )
            vpMapLines[i] = NULL;
        else
            vpMapLines[i] = mmpnIdx2MapLines[mpidx];
    }
    initkf.UndistortKeyLines();
#endif

    //use initkf to initialize
    KeyFrame* kf = new KeyFrame(initkf, this, KFDB, vpMapPoints, vpMapLines);
    kf->mnId = initkf.nId;
    kf->SetPose(Tcw);
    kf->ComputeBoW();
    for( int i = 0; i < initkf.N; ++i)
    {
       if( vpMapPoints[i] )
       {
           vpMapPoints[i]->AddObservation(kf,i);
           if( !vpMapPoints[i]->GetReferenceKeyFrame())
               vpMapPoints[i]->SetReferenceKeyFrame(kf);
       }
    }
#ifdef HasLine
    for( int i = 0; i < initkf.N_l; ++i)
    {
       if( vpMapLines[i] )
       {
           vpMapLines[i]->AddObservation(kf,i);
           if( !vpMapLines[i]->GetReferenceKeyFrame())
               vpMapLines[i]->SetReferenceKeyFrame(kf);
       }
    }
#endif
    if( KFDB != NULL )
        KFDB->add(kf);

    return kf;
}

void Map::Save(const string& filename)
{
    unique_lock<mutex> lock(mMutexMap);
    cout<<endl<<"Map Saving to "<<filename<<endl;
    ofstream f;
    
    f.open((filename + ".db").c_str(), ios_base::out|ios::binary);

    unsigned long int nMapPoints = mspMapPoints.size();
    cout << "The number of MapPoints: " << nMapPoints << endl;
    f.write((char*)&nMapPoints, sizeof(nMapPoints));
    for(auto mp:mspMapPoints)
        SaveMapPoint(f,mp);

#ifdef HasLine
    unsigned long int nMapLines = mspMapLines.size();
    cout << "The number of MapLines: " << nMapLines << endl;
    f.write((char*)&nMapLines, sizeof(nMapLines));
    for(auto ml:mspMapLines)
        SaveMapLine(f,ml);
#endif

    cerr<<"The number of KeyFrames: "<<mspKeyFrames.size()<<endl;
    unsigned long int nKeyFrames = mspKeyFrames.size();
    f.write((char*)&nKeyFrames,sizeof(nKeyFrames));

    for(auto kf:mspKeyFrames)
        SaveKeyFrame(f,kf);

    for(auto kf:mspKeyFrames)
    {
        KeyFrame* parent = kf->GetParent();
        unsigned long int parent_id = ULONG_MAX;
        if(parent)
            parent_id = parent->mnId;
        f.write((char*)&parent_id, sizeof(parent_id));
        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
        f.write((char*)&nb_con,sizeof(nb_con));
        for(auto ckf:kf->GetConnectedKeyFrames())
        {
            int weight = kf->GetWeight(ckf);
            f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            f.write((char*)&weight, sizeof(weight));
        }
    }

    f.close();
    cout<<"Map Saving Finished!"<<endl<<endl;
}

void Map::Load(const string& filename, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB)
{
    ifstream f;
    f.open((filename + ".db").c_str());
    if(!f.is_open()) {
        cout << "Can Not Open File " << filename << endl;
        throw f;
    }

    unsigned long int nMapPoints;
    f.read((char*)&nMapPoints, sizeof(nMapPoints));//read mappoint number
    cout<<"The number of MapPoints:"<<nMapPoints<<endl;

    mmpnIdx2MapPoints.clear();
    for(unsigned int i = 0;i < nMapPoints; ++i)//read every mappoint and join to map
    {
        MapPoint* mp = LoadMapPoint(f);
        AddMapPoint(mp);
        mmpnIdx2MapPoints[mp->mnId] = mp;
    }

#ifdef HasLine
    unsigned long int nMapLines;
    f.read((char*)&nMapLines, sizeof(nMapLines));//read mapline number
    cout<<"The number of MapLines:"<<nMapLines<<endl;

    for(unsigned int i = 0;i < nMapLines; ++i)//read every mapline and join to map
    {
        MapLine* ml = LoadMapLine(f);
        AddMapLine(ml);
        mmpnIdx2MapLines[ml->mnId] = ml;
    }
#endif

    unsigned long int nKeyFrames;
    f.read((char*)&nKeyFrames, sizeof(nKeyFrames));//read keyframe number
    cout<<"The number of KeyFrames:"<<nKeyFrames<<endl;

    vector<KeyFrame*> kf_by_order;
    for(unsigned int i = 0; i < nKeyFrames; ++i)
    {
        KeyFrame* kf = LoadKeyFrame(f, mySystemSetting, KFDB);
        AddKeyFrame(kf);
        kf_by_order.push_back(kf);
    }

    mmpnIdx2MapPoints.clear();
    mmpnIdx2MapLines.clear();

    map<unsigned long int, KeyFrame*> kf_by_id;
    for(auto kf: mspKeyFrames)
        kf_by_id[kf->mnId] = kf;
    for(auto kf:kf_by_order)
    {
        unsigned long int parent_id;//read parent id
        f.read((char*)&parent_id, sizeof(parent_id));


        if( parent_id != ULONG_MAX && parent_id<nKeyFrames)
            kf->ChangeParent(kf_by_id[parent_id]);

        //read now keyframe relationship
        unsigned long int nb_con;
        f.read((char*)&nb_con, sizeof(nb_con));

        //read every relate keyframe id & weight
        for(unsigned long int i = 0; i < nb_con; ++i)
        {
            unsigned long int id;
            int weight;
            f.read((char*)&id, sizeof(id));
            f.read((char*)&weight, sizeof(weight));
            if(id < nKeyFrames)
                kf->AddConnection(kf_by_id[id], weight);
        }
    }

    for( auto mp: mspMapPoints )
    {
        if(mp)
        {
            mp->ComputeDistinctiveDescriptors();
            mp->UpdateNormalAndDepth();
        }
    }
#ifdef HasLine
    for( auto ml: mspMapLines )
    {
        if(ml)
        {
            ml->ComputeDistinctiveDescriptors();
            ml->UpdateNormalAndDepth();
        }
    }
#endif
    f.close();
    unsigned long int MaxKFid = GetMaxKFid();
    unsigned long int MaxMPid = GetMaxMPid();
    unsigned long int MaxFid = GetMaxFid();

    KeyFrame::nNextId = MaxKFid + 1;
    MapPoint::nNextId = MaxMPid + 1;
    Frame::nNextId = MaxFid + 1;
}

} //namespace ORB_SLAM
