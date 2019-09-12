#ifndef SYSTEMSETTING_H
#define SYSTEMSETTING_H

#include <string>
#include "ORBVocabulary.h"
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{
    class SystemSetting
    {
    public:
        SystemSetting(ORBVocabulary* pORBVoc, LineVocabulary* pLineVoc);
        bool LoadSystemSetting(const std::string strSettingPath);
        ~SystemSetting();

        ORBVocabulary* pORBvocabulary;
        LineVocabulary* pLinevocabulary;

        float width;
        float height;
        float fx;
        float fy;
        float cx;
        float cy;
        float invfx;
        float invfy;
        float bf;
        float b;
        float fps;
        cv::Mat K;
        cv::Mat DistCoef;
        bool initialized;

        //camera feature parameters
        int nRGB;

        //ORB feature parameters
        int nFeatures;
        float fScaleFactor;
        int nLevels;
        float fIniThFAST;
        float fMinThFAST;

        //other
        float ThDepth = -1;
        float DepthMapFactor = -1;

    };
}
#endif
