/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#include <Config.h>
using namespace std;

Config::Config()
{
    bIsRelocalizationMode = false;
    path_LoadMap = "";
    path_SaveMap = "";

    img_width = 0;
    img_height = 0;

    // kf decision (SLAM) parameters
    min_entropy_ratio     = 0.85;
    max_kf_t_dist         = 5.0;
    max_kf_r_dist         = 15.0;

    // StVO-PL options
    // -----------------------------------------------------------------------------------------------------
    has_points         = true;      // true if using points
    has_lines          = true;      // true if using line segments
    use_fld_lines      = false;     // true if using FLD detector
    lr_in_parallel     = false;      // true if detecting and matching features in parallel
    pl_in_parallel     = false;      // true if detecting points and line segments in parallel
    best_lr_matches    = true;      // true if double-checking the matches between the two images
    adaptative_fast    = true;      // true if using adaptative fast_threshold
    use_motion_model   = true;     // true if using constant motion model

    // Tracking parameters
    // -----------------------------------------------------------------------------------------------------
    // Point features
    max_dist_epip     = 1.0;        // max. epipolar distance in pixels
    min_disp          = 1.0;        // min. disparity (avoid points in the infinite)
    min_ratio_12_p    = 0.75;        // min. ratio between the first and second best matches

    // Line segment features
    line_sim_th       = 0.75;       // threshold for cosine similarity
    stereo_overlap_th = 0.75;
    f2f_overlap_th    = 0.75;
    min_line_length   = 0.025;      // min. line length (relative to img size)
    line_horiz_th     = 0.1;        // parameter to avoid horizontal lines (pixels)
    min_ratio_12_l    = 0.9;        // parameter to avoid outliers in line matching
    ls_min_disp_ratio = 0.7;        // min ratio between min(disp)/max(disp) for a LS

    // Adaptative FAST parameters
    fast_min_th       = 5;          // min. value for FAST threshold
    fast_max_th       = 50;         // max. value for FAST threshold
    fast_inc_th       = 5;          // base increment for the FAST threshold
    fast_feat_th      = 50;         // base number of features to increase/decrease FAST threshold
    fast_err_th       = 0.5;        // threshold for the optimization error

    // Optimization parameters
    // -----------------------------------------------------------------------------------------------------
    homog_th         = 1e-7;        // avoid points in the infinite
    min_features     = 10;          // min. number of features to perform StVO
    max_iters        = 5;           // max. number of iterations in the first stage of the optimization
    max_iters_ref    = 10;          // max. number of iterations in the refinement stage
    min_error        = 1e-7;        // min. error to stop the optimization
    min_error_change = 1e-7;        // min. error change to stop the optimization
    inlier_k         = 4.0;         // factor to discard outliers before the refinement stage

    // Feature detection parameters
    // -----------------------------------------------------------------------------------------------------
    matching_strategy = 3;
    matching_s_ws     = 10;
    matching_f2f_ws   = 3;

    // ORB detector
    orb_nfeatures    = 1200;
    orb_scale_factor = 1.2;
    orb_nlevels      = 4;
    orb_edge_th      = 19;
    orb_wta_k        = 2;            // was set to 4
    orb_score        = 1;            // 0 - HARRIS  |  1 - FAST
    orb_patch_size   = 31;
    orb_fast_th      = 20;           // default FAST threshold
    // LSD parameters
    lsd_nfeatures    = 300;          // set to 0 if keeping all lines
    lsd_refine       = 0;
    lsd_scale        = 1.2;
    lsd_sigma_scale  = 0.6;
    lsd_quant        = 2.0;
    lsd_ang_th       = 22.5;
    lsd_log_eps      = 1.0;
    lsd_density_th   = 0.6;
    lsd_n_bins       = 1024;


        // SLAM parameters
    // -----------------------------------------------------------------------------------------------------
    fast_matching         = true;       // allow for the fast matching (window-based) of the map features
    has_refinement        = false;      // refine the pose between keyframes (disabled as it is also performed by the LBA)
    mutithread_slam       = false;       // if true the system runs with both the VO, LBA and LC in parallel threads

    // lm numbers and errors
    min_lm_obs            = 5;          // min number of observations for a landmark to be considered as inlier
    max_common_fts_kf     = 0.9;        // max number of common features for a keyframe to be considered redundant (disabled)

    max_kf_epip_p         = 1.0;
    max_kf_epip_l         = 1.0;

    max_lm_3d_err         = 0.1;
    max_lm_dir_err        = 0.1;
    max_point_point_error = 0.1;
    max_point_line_error  = 0.1;
    max_dir_line_error    = 0.1;

    // graphs parameters
    min_lm_ess_graph      = 150;
    min_lm_cov_graph      = 75;
    min_kf_local_map      = 3;

    // LBA
    lambda_lba_lm         = 0.00001;
    lambda_lba_k          = 10.0;
    max_iters_lba         = 15;

    // Loop closure
    vocabulary_p          = "";
    vocabulary_l          = "";

    lc_mat                = 0.5;
    lc_res                = 1.5;
    lc_unc                = 0.01;
    lc_inl                = 0.3;
    lc_trs                = 1.5;
    lc_rot                = 35.0;

    max_iters_pgo         = 100;
    lc_kf_dist            = 50;
    lc_kf_max_dist        = 50;
    lc_nkf_closest        = 4;
    lc_inlier_ratio       = 30.0;

    min_pt_matches        = 10;
    min_ls_matches        = 6;
    kf_inlier_ratio       = 30.0;
}

Config::~Config(){}

Config& Config::getInstance()
{
    static Config instance; // Instantiated on first use and guaranteed to be destroyed
    return instance;
}

template<typename T> inline T loadSafe(const cv::FileStorage &fSettings, std::string param, T default_value = T())
{
    cv::FileNode node = fSettings[param];
    if(node.type()!=0)
        return static_cast<T>(node);
    else
        return default_value;
}

bool loadSafe(const cv::FileStorage &fSettings, std::string param, bool default_value)
{
    cv::FileNode node = fSettings[param];
    if(node.type()!=0)
    {
        if(node.type()==cv::FileNode::INT) {
            int value = static_cast<int>(node);
            return value==1?true:false;
        }
        else if(node.type()==cv::FileNode::STR) {
            std::string value =  static_cast<std::string>(node);
            if(value.substr(0,4)=="true") return true;
            else if(value.substr(0,5)=="false") return false;
            else return default_value;
        }
        else return default_value;
    }
    else
        return default_value;
}

void Config::loadFromFile( const string &strSettingPath )
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    Config::isRelocalizationMode() = loadSafe(fSettings, "RelocalizationMode", Config::isRelocalizationMode());
    Config::pathLoadMap() = loadSafe(fSettings, "PathLoadMap", Config::pathLoadMap());
    Config::pathSaveMap() = loadSafe(fSettings, "PathSaveMap", Config::pathSaveMap());

    Config::imgWidth() = loadSafe(fSettings, "Camera.width", Config::imgWidth());
    Config::imgHeight() = loadSafe(fSettings, "Camera.height", Config::imgHeight());

    Config::minEntropyRatio() = loadSafe(fSettings, "min_entropy_ratio", Config::minEntropyRatio());
    Config::maxKFTDist() = loadSafe(fSettings, "max_kf_t_dist", Config::maxKFTDist());
    Config::maxKFRDist() = loadSafe(fSettings, "max_kf_r_dist", Config::maxKFRDist());

    Config::hasPoints() = loadSafe(fSettings, "has_points", Config::hasPoints());
    Config::hasLines() = loadSafe(fSettings, "has_lines", Config::hasLines());
    Config::useFLDLines() = loadSafe(fSettings, "use_fld_lines", Config::useFLDLines());
    Config::lrInParallel() = loadSafe(fSettings, "lr_in_parallel", Config::lrInParallel());
    Config::plInParallel() = loadSafe(fSettings, "pl_in_parallel", Config::plInParallel());
    Config::bestLRMatches() = loadSafe(fSettings, "best_lr_matches", Config::bestLRMatches());
    Config::adaptativeFAST() = loadSafe(fSettings, "adaptative_fast", Config::adaptativeFAST());
    Config::useMotionModel() = loadSafe(fSettings, "use_motion_model", Config::useMotionModel());

    Config::maxDistEpip() = loadSafe(fSettings, "max_dist_epip", Config::maxDistEpip());
    Config::minDisp() = loadSafe(fSettings, "min_disp", Config::minDisp());
    Config::minRatio12P() = loadSafe(fSettings, "min_ratio_12_p", Config::minRatio12P());

    Config::lineSimTh() = loadSafe(fSettings, "line_sim_th", Config::lineSimTh());
    Config::stereoOverlapTh() = loadSafe(fSettings, "stereo_overlap_th", Config::stereoOverlapTh());
    Config::f2fOverlapTh() = loadSafe(fSettings, "f2f_overlap_th", Config::f2fOverlapTh());
    Config::minLineLength() = loadSafe(fSettings, "min_line_length", Config::minLineLength());
    Config::lineHorizTh() = loadSafe(fSettings, "line_horiz_th", Config::lineHorizTh());
    Config::minRatio12L() = loadSafe(fSettings, "min_ratio_12_l", Config::minRatio12L());
    Config::lsMinDispRatio() = loadSafe(fSettings, "ls_min_disp_ratio", Config::lsMinDispRatio());

    Config::fastMinTh() = loadSafe(fSettings, "fast_min_th", Config::fastMinTh());
    Config::fastMaxTh() = loadSafe(fSettings, "fast_max_th", Config::fastMaxTh());
    Config::fastIncTh() = loadSafe(fSettings, "fast_inc_th", Config::fastIncTh());
    Config::fastFeatTh() = loadSafe(fSettings, "fast_feat_th", Config::fastFeatTh());
    Config::fastErrTh() = loadSafe(fSettings, "fast_err_th", Config::fastErrTh());

    Config::rgbdMinDepth() = loadSafe(fSettings, "rgbd_min_depth", Config::rgbdMinDepth());
    Config::rgbdMaxDepth() = loadSafe(fSettings, "rgbd_max_depth", Config::rgbdMaxDepth());

    Config::homogTh() = loadSafe(fSettings, "homog_th", Config::homogTh());
    Config::minFeatures() = loadSafe(fSettings, "min_features", Config::minFeatures());
    Config::maxIters() = loadSafe(fSettings, "max_iters", Config::maxIters());
    Config::maxItersRef() = loadSafe(fSettings, "max_iters_ref", Config::maxItersRef());
    Config::minError() = loadSafe(fSettings, "min_error", Config::minError());
    Config::minErrorChange() = loadSafe(fSettings, "min_error_change", Config::minErrorChange());
    Config::inlierK() = loadSafe(fSettings, "inlier_k", Config::inlierK());

    Config::matchingStrategy() = loadSafe(fSettings, "matching_strategy", Config::matchingStrategy());
    Config::matchingSWs() = loadSafe(fSettings, "matching_s_ws", Config::matchingSWs());
    Config::matchingF2FWs() = loadSafe(fSettings, "matching_f2f_ws", Config::matchingF2FWs());

    Config::orbNFeatures() = loadSafe(fSettings, "orb_nfeatures", Config::orbNFeatures());
    Config::orbScaleFactor() = loadSafe(fSettings, "orb_scale_factor", Config::orbScaleFactor());
    Config::orbNLevels() = loadSafe(fSettings, "orb_nlevels", Config::orbNLevels());
    Config::orbEdgeTh() = loadSafe(fSettings, "orb_edge_th", Config::orbEdgeTh());
    Config::orbWtaK() = loadSafe(fSettings, "orb_wta_k", Config::orbWtaK());
    Config::orbScore() = loadSafe(fSettings, "orb_score", Config::orbScore());
    Config::orbPatchSize() = loadSafe(fSettings, "orb_patch_size", Config::orbPatchSize());
    Config::orbFastTh() = loadSafe(fSettings, "orb_fast_th", Config::orbFastTh());

    Config::lsdNFeatures() = loadSafe(fSettings, "lsd_nfeatures", Config::lsdNFeatures());
    Config::lsdRefine() = loadSafe(fSettings, "lsd_refine", Config::lsdRefine());
    Config::lsdScale() = loadSafe(fSettings, "lsd_scale", Config::lsdScale());
    Config::lsdSigmaScale() = loadSafe(fSettings, "lsd_sigma_scale", Config::lsdSigmaScale());
    Config::lsdQuant() = loadSafe(fSettings, "lsd_quant", Config::lsdQuant());
    Config::lsdAngTh() =loadSafe(fSettings, "lsd_ang_th", Config::lsdAngTh());
    Config::lsdLogEps() = loadSafe(fSettings, "lsd_log_eps", Config::lsdLogEps());
    Config::lsdDensityTh() = loadSafe(fSettings, "lsd_density_th", Config::lsdDensityTh());
    Config::lsdNBins() = loadSafe(fSettings, "lsd_n_bins", Config::lsdNBins());


    Config::minLMObs() = loadSafe(fSettings, "min_lm_obs", Config::minLMObs());
    Config::maxCommonFtsKF() = loadSafe(fSettings, "max_common_fts_kf", Config::maxCommonFtsKF());

    Config::maxKFEpipP() = loadSafe(fSettings, "max_kf_epip_p", Config::maxKFEpipP());
    Config::maxKFEpipL() = loadSafe(fSettings, "max_kf_epip_l", Config::maxKFEpipL());

    Config::maxLM3DErr() = loadSafe(fSettings, "max_lm_3d_err", Config::maxLM3DErr());
    Config::maxLMDirErr() = loadSafe(fSettings, "max_lm_dir_err", Config::maxLMDirErr());
    Config::maxPointPointError() = loadSafe(fSettings, "max_point_point_error", Config::maxPointPointError());
    Config::maxPointLineError() = loadSafe(fSettings, "max_point_line_error", Config::maxPointLineError());
    Config::maxDirLineError() = loadSafe(fSettings, "max_dir_line_error", Config::maxDirLineError());

    Config::minLMEssGraph() = loadSafe(fSettings, "min_lm_ess_graph", Config::minLMEssGraph());
    Config::minLMCovGraph() = loadSafe(fSettings, "min_lm_cov_graph", Config::minLMCovGraph());
    Config::minKFLocalMap() = loadSafe(fSettings, "min_kf_local_map", Config::minKFLocalMap());

    Config::lambdaLbaLM() = loadSafe(fSettings, "lambda_lba_lm", Config::lambdaLbaLM());
    Config::lambdaLbaK() = loadSafe(fSettings, "lambda_lba_k", Config::lambdaLbaK());
    Config::maxItersLba() = loadSafe(fSettings, "max_iters_lba", Config::maxItersLba());

    Config::dbowVocP() = loadSafe(fSettings, "vocabulary_p", Config::dbowVocP());
    Config::dbowVocL() = loadSafe(fSettings, "vocabulary_l", Config::dbowVocL());
    // Config::dbowVocP() = fSettings["vocabulary_p"].string();
    // Config::dbowVocL() = std::string(fSettings["vocabulary_l"]);

    Config::lcMat() = loadSafe(fSettings, "lc_mat", Config::lcMat());
    Config::lcRes() = loadSafe(fSettings, "lc_res", Config::lcRes());
    Config::lcUnc() = loadSafe(fSettings, "lc_unc", Config::lcUnc());
    Config::lcInl() = loadSafe(fSettings, "lc_inl", Config::lcInl());
    Config::lcTrs() = loadSafe(fSettings, "lc_trs", Config::lcTrs());
    Config::lcRot() = loadSafe(fSettings, "lc_rot", Config::lcRot());

    Config::maxItersPGO() = loadSafe(fSettings, "max_iters_pgo", Config::maxItersPGO());
    Config::lcKFDist() = loadSafe(fSettings, "lc_kf_dist", Config::lcKFDist());
    Config::lcKFMaxDist() = loadSafe(fSettings, "lc_kf_max_dist", Config::lcKFMaxDist());
    Config::lcNKFClosest() = loadSafe(fSettings, "lc_nkf_closest", Config::lcNKFClosest());
    Config::lcInlierRatio() = loadSafe(fSettings, "lc_inlier_ratio", Config::lcInlierRatio());

    Config::minPointMatches() = loadSafe(fSettings, "min_pt_matches", Config::minPointMatches());
    Config::minLineMatches() = loadSafe(fSettings, "min_ls_matches", Config::minLineMatches());
    Config::kfInlierRatio() = loadSafe(fSettings, "kf_inlier_ratio", Config::kfInlierRatio());

    Config::fastMatching() = loadSafe(fSettings, "fast_matching", Config::fastMatching());
    Config::hasRefinement() = loadSafe(fSettings, "has_refinement", Config::hasRefinement());
    Config::multithreadSLAM() = loadSafe(fSettings, "mutithread_slam", Config::multithreadSLAM());
}
