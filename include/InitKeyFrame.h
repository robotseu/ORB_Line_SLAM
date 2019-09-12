#ifndef INITKF_H
#define INITKF_H

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "SystemSetting.h"
#include <opencv2/opencv.hpp>
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Frame.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ORBextractor.h"
#include "LineExtractor.h"

namespace ORB_SLAM2
{
	#define FRAME_GRID_ROWS 48
	#define FRAME_GRID_COLS 64

	class SystemSetting;
	class KeyFrameDatabase;

	class InitKeyFrame
	{
	public:

		// InitKeyFrame(const Frame &frame);
		InitKeyFrame(SystemSetting &SS);

		void UndistortKeyPoints();
		void UndistortKeyLines();
		bool PosInGrid(const cv::KeyPoint& kp, int &posX, int &posY);
		void AssignFeaturesToGrid();

	public:
		ORBVocabulary* pORBvocabulary;
    	LineVocabulary* pLinevocabulary;

		long unsigned int nId;
		long unsigned int mnFrameId;
		double TimeStamp;

		float fGridElementWidthInv;
		float fGridElementHeightInv;
		std::vector<std::size_t> vGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

		float fx;
		float fy;
		float cx;
		float cy;
		float invfx;
		float invfy;
		float bf;
		float b;
		float ThDepth;
		int N;
		int N_l;
		std::vector<cv::KeyPoint> vKps;
		std::vector<cv::KeyPoint> vKpsUn;
		cv::Mat Descriptors;

		std::vector<float> vRight;
		std::vector<float> vDepth;

		DBoW2::BowVector BowVec;
		DBoW2::FeatureVector FeatVec;

    	std::vector<cv::line_descriptor::KeyLine> vKeys_Line;
    	std::vector<cv::line_descriptor::KeyLine> vKeysUn_Line;
    	std::vector<pair<float,float>> vDisparity_l;
    	std::vector<Vector3d> vle_l;		
    	cv::Mat Descriptors_Line;

		//DBoW2::BowVector BowVec_l;
		//DBoW2::FeatureVector FeatVec_l;

		int nScaleLevels;
		float fScaleFactor;
		float fLogScaleFactor;
		std::vector<float> vScaleFactors;
		std::vector<float> vLevelSigma2;
		std::vector<float> vInvLevelSigma2;
		std::vector<float> vInvScaleFactors;

		int nMinX;
		int nMinY;
		int nMaxX;
		int nMaxY;
		cv::Mat K;
		cv::Mat DistCoef;
	};
}

#endif
