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

#ifndef LINEEXTRACTOR_H
#define LINEEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>

#include "Auxiliar.h"
#include "Config.h"

// #include <opencv2/ximgproc.hpp>
// #include <opencv2/ximgproc/fast_line_detector.hpp>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>

namespace ORB_SLAM2
{

class Lineextractor
{
public:
    Lineextractor(int _lsd_nfeatures, double _llength_th, bool _bFLD = false);
    Lineextractor(int _lsd_nfeatures, double _llength_th, int _lsd_refine, double _lsd_scale, double _lsd_sigma_scale, 
    double _lsd_quant, double _lsd_ang_th, double _lsd_log_eps, double _lsd_density_th, int _lsd_n_bins, bool _bFLD = false);

    ~Lineextractor(){}

    void operator()( const cv::Mat& image, const cv::Mat& mask,
      std::vector<cv::line_descriptor::KeyLine>& keylines,
      cv::Mat& descriptors_line);

protected:
    int    lsd_nfeatures;
    double min_line_length;

    // lines detection and matching
    int    lsd_refine;
    double lsd_scale;
    double lsd_sigma_scale;
    double lsd_quant;
    double lsd_ang_th;
    double lsd_log_eps;
    double lsd_density_th;
    int    lsd_n_bins;
    // double line_horiz_th;
    
    bool   bFLD;
    
    double min_ratio_12_l;
    double ls_min_disp_ratio;
};

} //namespace ORB_SLAM

#endif

