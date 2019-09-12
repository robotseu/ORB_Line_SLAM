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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KDTree.h"

#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;

class point_kdtree : public std::array<float, 2>
{
public:
    static const int DIM = 2;       // dimension of space (or "k" of k-d tree)

    point_kdtree() { (*this)[0]=0; (*this)[1]=0; r=0; }
    point_kdtree(float x, float y, float _r=0) { (*this)[0]=x; (*this)[1]=y; r=_r; }
    point_kdtree(const cv::KeyPoint& kp) : point_kdtree(kp.pt, kp.size) { }
    point_kdtree(const cv::Point2f& p, float _r=0) { (*this)[0]=p.x; (*this)[1]=p.y; r=_r; }

    float radius() { return r; }

private:
    float r;
};



class KeyFrameDatabase
{
    typedef unsigned int WordId;
public:

    KeyFrameDatabase(const ORBVocabulary &voc);
    KeyFrameDatabase(const ORBVocabulary &voc, const LineVocabulary &voc_l);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);
   std::vector<KeyFrame*> DetectLoopCandidatesWithLine(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);
   std::vector<KeyFrame*> DetectRelocalizationCandidatesWithLine(Frame* F);

protected:

  // Associated vocabulary
   const ORBVocabulary* mpVoc;
   const LineVocabulary* mpVoc_l;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;
  std::vector<list<KeyFrame*> > mvInvertedFile_l;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
