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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include"Frame.h"
#include"Map.h"
#include"ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include"LineExtractor.h"
#include"KeyFrame.h"

using namespace std;
using namespace ORB_SLAM2;

void LoadImages_kitti(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
void LoadImages_EuRoC(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);
void LoadImages_oxford(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    if(argc == 3)
        LoadImages_kitti(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);
    else if(argc == 4)
        LoadImages_oxford(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);
    else if(argc == 6)
        LoadImages_EuRoC(string(argv[3]), string(argv[4]), string(argv[5]), vstrImageLeft, vstrImageRight, vTimestamps);
    else {
        cerr << endl << "Usage: ./placeRecognition path_to_settings path_to_sequence [imgNum]" << endl;
        return 1;
    }

    const int nImages = vstrImageLeft.size();

    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   


    ORB_SLAM2::Map* mpMap = new ORB_SLAM2::Map();

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    ORBVocabulary* mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile("/home/lab404/Software/qt_workspace/ORB_Line_SLAM/Vocabulary/ORBvoc.txt");
    LineVocabulary* mpVocabulary_l = new LineVocabulary();
    bool bVocLoad_l = mpVocabulary_l->loadFromTextFile("/home/lab404/Software/qt_workspace/ORB_Line_SLAM/Vocabulary/LSDvoc.txt");     //Config::dbowVocL()
    if(!bVocLoad || !bVocLoad_l)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open." << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    KeyFrameDatabase* mpKeyFrameDB = new KeyFrameDatabase(*mpVocabulary, *mpVocabulary_l);


    // Load camera parameters from settings file
    cv::FileStorage fSettings(argv[1], cv::FileStorage::READ);
    cv::Mat mK;
    cv::Mat mDistCoef;
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    float mbf = fSettings["Camera.bf"];

    int nRGB = fSettings["Camera.RGB"];
    bool mbRGB = nRGB;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // Load Line parameters
    int lsd_nfeatures = fSettings["lsd_nfeatures"];
    double min_line_length = fSettings["min_line_length"];
    int lsd_refine = fSettings["lsd_refine"];
    double lsd_scale = fSettings["lsd_scale"];
    double lsd_sigma_scale = fSettings["lsd_sigma_scale"];
    double lsd_quant = fSettings["lsd_quant"];
    double lsd_ang_th = fSettings["lsd_ang_th"];
    double lsd_log_eps = fSettings["lsd_log_eps"];
    double lsd_density_th = fSettings["lsd_density_th"];
    int lsd_n_bins = fSettings["lsd_n_bins"];

    ORBextractor* mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    Lineextractor* mpLineextractorLeft = new Lineextractor(lsd_nfeatures, min_line_length, lsd_refine,
                                    lsd_scale, lsd_sigma_scale, lsd_quant, lsd_ang_th, lsd_log_eps, lsd_density_th, lsd_n_bins, false);

    ORBextractor* mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    Lineextractor* mpLineextractorRight = new Lineextractor(lsd_nfeatures, min_line_length, lsd_refine,
                                    lsd_scale, lsd_sigma_scale, lsd_quant, lsd_ang_th, lsd_log_eps, lsd_density_th, lsd_n_bins, false);

    float mThDepth = mbf*(float)fSettings["ThDepth"]/fx;


//    ofstream fout("/home/lab404/Documents/ORB_Line_SLAM/placeRecognition_Loop.txt");
//    ofstream fout("/home/lab404/Documents/ORB_Line_SLAM/placeRecognition_LoopWithLine.txt");
//    ofstream fout("/home/lab404/Documents/ORB_Line_SLAM/placeRecognition_Reloc.txt");
//    ofstream fout("/home/lab404/Documents/ORB_Line_SLAM/placeRecognition_RelocWithLine.txt");
    ofstream fout("/home/lab404/Documents/ORB_Line_SLAM/placeRecognition.txt");

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        // Pass the images to the SLAM system
        
        cv::Mat mImGray = imLeft;
        cv::Mat imGrayRight = imRight;

        if(mImGray.channels()==3)
        {
            if(mbRGB)
            {
                cvtColor(mImGray,mImGray,CV_RGB2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray,mImGray,CV_BGR2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
            }
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
            {
                cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
            }
            else
            {
                cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
            }
        }

        Frame mCurrentFrame = Frame(mImGray,imGrayRight,tframe,mpORBextractorLeft,mpORBextractorRight,
            mpLineextractorLeft,mpLineextractorRight,mpVocabulary,mpVocabulary_l,
            mK,mDistCoef,mbf,mThDepth);
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
        mCurrentFrame.ComputeBoW();

        KeyFrame* pCurrentKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        pCurrentKF->ComputeBoW();

//        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(pCurrentKF,0);
//        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidatesWithLine(pCurrentKF,0);
//        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidatesWithLine(&mCurrentFrame);
        for(auto it=vpCandidateKFs.begin(); it!=vpCandidateKFs.end();)
        {
            if(ni-(*it)->mnId<20) it = vpCandidateKFs.erase(it);
            else ++it;
        }

        if(!vpCandidateKFs.empty())
        {
            cout<<"currID:\t "<<pCurrentKF->mnId<<endl;
            cout<<"loopID:\t";
            fout<<"currID:\t "<<pCurrentKF->mnId<<endl;
            fout<<"loopID:\t";
            int iImg = 0;
            for(auto it=vpCandidateKFs.begin(),itend=vpCandidateKFs.end(); it!=itend; ++it)
            {
                int loopId = (*it)->mnId;
                // if(ni-loopId<5) continue;
                cout<<" "<<loopId;
                fout<<" "<<loopId;
                cv::Mat imLoop = cv::imread(vstrImageLeft[loopId],CV_LOAD_IMAGE_UNCHANGED);
                //cv::imshow("loop image"+to_string(iImg++),imLoop);
            }
            cout<<endl<<endl;
            fout<<endl<<endl;
        }

        if(false)
        {
            cv::Mat imPL = imLeft.clone();
            if(imPL.channels()<3) //this should be always true
                cvtColor(imPL,imPL,CV_GRAY2BGR);

            int N = pCurrentKF->N;
            int N_l = pCurrentKF->N_l;
            const vector<cv::KeyPoint>& vKeysUn = pCurrentKF->mvKeysUn;
            const vector<cv::line_descriptor::KeyLine>& vKeysUn_Line = pCurrentKF->mvKeysUn_Line;

            random_device rnd_dev;
            mt19937 rnd(rnd_dev());
            uniform_int_distribution<int> color_dist(0, 255);
            for(int i=0; i<N; ++i)
            {
                cv::Scalar color = Scalar(color_dist(rnd), color_dist(rnd), color_dist(rnd));
                cv::Point2f pt = vKeysUn[i].pt;
                cv::circle(imPL, pt, 2, color, -1);         // Green solid
            }
            for(int i=0; i<N_l; ++i)
            {
                cv::Scalar color = Scalar(color_dist(rnd), color_dist(rnd), color_dist(rnd));
                cv::Point2f spt(vKeysUn_Line[i].startPointX,vKeysUn_Line[i].startPointY);
                cv::Point2f ept(vKeysUn_Line[i].endPointX,vKeysUn_Line[i].endPointY);
                cv::line(imPL, spt, ept, color, 1.5);         // Red
            }
            cv::imshow("imgPL",imPL);

            vector<point_kdtree> vpkdt;
            vpkdt.resize(N);
            for(int i=0; i<N; ++i)
                vpkdt[i] = point_kdtree(vKeysUn[i]);

            cv::Mat im = imLeft.clone();
            if(im.channels()<3) //this should be always true
                cvtColor(im,im,CV_GRAY2BGR);
            KDTree<point_kdtree> kdtree(vpkdt);
            for(int i=0; i<N_l; ++i)
            {

                point_kdtree midpoint(vKeysUn_Line[i].pt);
                vector<int> radIndices = kdtree.knnSearch(midpoint, 5);
                for(auto it=radIndices.begin(); it!=radIndices.end(); ++it)
                {
                    Vector3d sp_l; sp_l << vKeysUn_Line[i].startPointX, vKeysUn_Line[i].startPointY, 1.0;
                    Vector3d ep_l; ep_l << vKeysUn_Line[i].endPointX,   vKeysUn_Line[i].endPointY,   1.0;
                    Vector3d le_l; le_l << sp_l.cross(ep_l); le_l = le_l / std::sqrt( le_l(0)*le_l(0) + le_l(1)*le_l(1) );

                    cv::Point2f pt_pair = vKeysUn[*it].pt;
                    float dist = le_l(0) * pt_pair.x + le_l(1) * pt_pair.y + le_l(2);
                    if(fabs(dist)<vKeysUn[*it].size) {
                        cv::Scalar color = Scalar(color_dist(rnd), color_dist(rnd), color_dist(rnd));
                        cv::circle(im, pt_pair, vKeysUn[*it].size, color, 2);
                        cv::circle(im, pt_pair, 2, color, -1);
                        cv::line(im, cv::Point2f(vKeysUn_Line[i].startPointX,vKeysUn_Line[i].startPointY),
                                 cv::Point2f(vKeysUn_Line[i].endPointX,vKeysUn_Line[i].endPointY),
                                 color, 2);
                        break;
                    }
                }
            }
            cv::imshow("wordpair img",im);
            cv::imwrite("/home/lab404/Documents/wordpairs/"+to_string(pCurrentKF->mnId)+".jpg",im);
            cv::waitKey(1e3);
//            for(int i=0; i<N; ++i)
//            {
//                cv::Mat im = imLeft.clone();
//                if(im.channels()<3) //this should be always true
//                    cvtColor(im,im,CV_GRAY2BGR);

//                vector<int> radIndices = kdtree.radiusSearch(vpkdt[i], vpkdt[i].radius());
//                cv::Point2f pt = vKeysUn[i].pt;
//                for(auto it=radIndices.begin(); it!=radIndices.end(); ++it)
//                {
//                    cv::Point2f pt_pair = vKeysUn[*it].pt;
//                    cv::circle(im, pt, vKeysUn[i].size, cv::Scalar(0,255,0), 1.5);         // Green
//                    cv::line(im, pt, pt_pair, cv::Scalar(0,0,255), 1.5);         // Red
//                }
//                //cv::imshow("wordpair img",im);
//                //cv::waitKey(1e3/3);
//            }

        }

        mpKeyFrameDB->add(pCurrentKF);
    }
    fout.close();

    return 0;
}

void LoadImages_kitti(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
    cout<<"--> Image starts from "<<vstrImageLeft[0]<<endl;
}
void LoadImages_EuRoC(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
void LoadImages_oxford(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    std::string cmd = "ls "+strPathToSequence+" -l | grep \"^-\" | wc -l";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return;
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);

    int n = std::stoi(result);
    cout<<"Img Number is "<<n<<endl;
    const int nTimes = n;
    vTimestamps.resize(nTimes);
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    string strPrefixLeft = strPathToSequence;
    string strPrefixRight = strPathToSequence;

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(4) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".jpg";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".jpg";
    }
    cout<<"--> Image starts from "<<vstrImageLeft[0]<<endl;
}
