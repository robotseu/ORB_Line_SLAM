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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "InitKeyFrame.h"
#include "SystemSetting.h"
#include "Converter.h"
#include <set>

#include <mutex>

#define HasLine

namespace ORB_SLAM2
{

class MapPoint;
class MapLine;
class KeyFrame;
class KeyFrameDatabase;
class SystemSetting;
class InitKeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void AddMapLine(MapLine* pML);
    void EraseMapPoint(MapPoint* pMP);
    void EraseMapLine(MapLine* pML);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void SetReferenceMapLines(const std::vector<MapLine*> &vpMLs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapLine*> GetAllMapLines();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<MapLine*> GetReferenceMapLines();

    long unsigned int MapPointsInMap();
    long unsigned int MapLinesInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();
    long unsigned int GetMaxMPid();
    long unsigned int GetMaxFid();

    void clear();

	void Save(const string& filename);
    void Load(const string& filename, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB);
    // 分段建图的载入
    void Load(const string& filename, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB, Eigen::Isometry3d transforms, int &MapPointCount, int &KeyFrameCount, int &FrameCount, int &FrameNow);
    void EraseUnObs();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    std::mutex mMutexLineCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<MapPoint*> mspMapPoints_each;
    std::set<MapLine*> mspMapLines;
    std::set<KeyFrame*> mspKeyFrames;

	std::map<unsigned long int, MapPoint*> mmpnIdx2MapPoints;
	std::map<unsigned long int, MapLine*> mmpnIdx2MapLines;
    std::map<unsigned long int, unsigned long int>mNewId2OldId;

    std::vector<MapPoint*> mvpReferenceMapPoints;
    std::vector<MapLine*> mvpReferenceMapLines;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    void SaveMapPoint(ofstream& f, MapPoint* mp);
    MapPoint* LoadMapPoint(ifstream& f);
    void SaveMapLine(ofstream& f, MapLine* ml);
    MapLine* LoadMapLine(ifstream& f);
    void SaveKeyFrame(ofstream& f, KeyFrame* kf);
    KeyFrame* LoadKeyFrame(ifstream& f, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB);
    KeyFrame* LoadKeyFrame( ifstream &f, SystemSetting* mySystemSetting, KeyFrameDatabase* KFDB, Eigen::Isometry3d transforms, int &MapPointCount, int &KeyFrameCount, int &FrameCount, int &FrameNow);
    void GetMapPointsIdx();
};

} //namespace ORB_SLAM

#endif // MAP_H
