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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{


void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);


    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }    

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

bool Optimizer::isGoodSolution( Matrix4d DT, Matrix6d DTcov, double err )
{
    SelfAdjointEigenSolver<Matrix6d> eigensolver(DTcov);
    Vector6d DT_cov_eig = eigensolver.eigenvalues();

    if( DT_cov_eig(0)<0.0 || DT_cov_eig(5)>1.0 || err < 0.0 || err > 1.0 || !is_finite(DT) )
    // if( DT_cov_eig(0)<-1e-30 || DT_cov_eig(5)>1.0 || err < 0.0 || err > 1.0 || !is_finite(DT) )
    {
        cout << "Not a good solution:" <<DT_cov_eig(0) << " \t" << DT_cov_eig(5) << " \t" << err << endl;
        return false;
    }

    return true;
}

void Optimizer::removeOutliers(Matrix4d DT, Frame *pFrame)
{

    //TODO: if not usig mad stdv, use just a fixed threshold (sqrt(7.815)) to filter outliers (with a single for loop...)
    Matrix3d DR = DT.block<3,3>(0,0);
    Vector3d Dt = DT.col(3).head(3);

    {
    unique_lock<mutex> lockP(MapPoint::mGlobalMutex);
    unique_lock<mutex> lockL(MapLine::mGlobalMutex);

    if (Config::hasPoints()) {
        // point features
        const int NP = pFrame->mvpMapPoints.size();
        vector<double> res_p;
        vector<int> res_p_idx;
        res_p.reserve(NP);
        res_p_idx.reserve(NP);
        for(int i=0; i<NP; ++i)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                Vector3d P_lastframe = pFrame->mv3DpointInPrevFrame[i];
                Vector3d P_ = DR*P_lastframe+Dt;
                Vector2d pl_proj = pFrame->projection( P_ );

                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                Vector2d pl_obs;
                pl_obs << kpUn.pt.x, kpUn.pt.y;
                res_p.push_back( ( pl_proj - pl_obs ).norm() * sqrt(pFrame->mvInvLevelSigma2[kpUn.octave]) );
                res_p_idx.push_back(i);
                //res_p.push_back( ( pl_proj - (*it)->pl_obs ).norm() );
            }
        }
        // estimate robust parameters
        double p_stdv, p_mean, inlier_th_p;
        vector_mean_stdv_mad( res_p, p_mean, p_stdv );
        inlier_th_p = Config::inlierK() * p_stdv;
        // inlier_th_p = sqrt(7.815);
        //cout << endl << p_mean << " " << p_stdv << "\t" << inlier_th_p << endl;
        // filter outliers
        for(int j=0, Nres=res_p_idx.size(); j<Nres; ++j)
        {
            int i = res_p_idx[j];
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                // if(res_p[j] > inlier_th_p)
                if(fabs(res_p[j]-p_mean) > inlier_th_p)
                {
                    if(pFrame->mvbOutlier[i]) continue;
                    pFrame->mvbOutlier[i] = true;
                    pFrame->n_inliers--;
                    pFrame->n_inliers_pt--;
                }
                else if(pFrame->mvbOutlier[i])          //convert from outlier to inlier
                {
                    pFrame->mvbOutlier[i] = false;
                    pFrame->n_inliers++;
                    pFrame->n_inliers_pt++;
                }
            }
        }
    }

    if (Config::hasLines()) {
        // line segment features
        const int NL = pFrame->mvpMapLines.size();
        vector<double> res_l;
        vector<int> res_l_idx;
        res_l.reserve(NL);
        res_l_idx.reserve(NL);
        for(int i=0; i<NL; ++i)
        {
            MapLine* pML = pFrame->mvpMapLines[i];
            if(pML)
            {
            //LineFeature* obs = pFrame->stereo_ls[i];
                Vector3d l_obs = pFrame->mvle_l[i]; //obs->le_obs;

                Vector3d sP_lastframe = pFrame->mv3DlineInPrevFrame[i].first;
                Vector3d sP_ = DR*sP_lastframe+Dt;
                Vector3d eP_lastframe = pFrame->mv3DlineInPrevFrame[i].second;
                Vector3d eP_ = DR*eP_lastframe+Dt;
                Vector2d spl_proj = pFrame->projection( sP_ );
                Vector2d epl_proj = pFrame->projection( eP_ );

                // projection error
                Vector2d err_li;
                err_li(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
                err_li(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
                res_l.push_back( err_li.norm() * sqrt(pFrame->mvInvLevelSigma2[pFrame->mvKeysUn_Line[i].octave]) );
                //res_l.push_back( err_li.norm() );
                res_l_idx.push_back(i);
            }
        }

        // estimate robust parameters
        double l_stdv, l_mean, inlier_th_l;
        vector_mean_stdv_mad( res_l, l_mean, l_stdv );
        inlier_th_l = Config::inlierK() * l_stdv;
        // inlier_th_l = sqrt(7.815);//sqrt(12.59);
        //cout << endl << l_mean << " " << l_stdv << "\t" << inlier_th_l << endl << endl;

        // filter outliers
        for(int j=0, Nres=res_l_idx.size(); j<Nres; ++j)
        {
            int i = res_l_idx[j];
            MapLine* pML = pFrame->mvpMapLines[i];
            if(pML)
            {
                // if(res_l[j] > inlier_th_l)
                if(fabs(res_l[j]-l_mean) > inlier_th_l)
                {
                    if(pFrame->mvbOutlier_Line[i]) continue;
                    pFrame->mvbOutlier_Line[i] = true;
                    pFrame->n_inliers--;
                    pFrame->n_inliers_ls--;
                }
                else if(pFrame->mvbOutlier_Line[i])     //convert from outlier to inlier
                {
                    pFrame->mvbOutlier_Line[i] = false;
                    pFrame->n_inliers++;
                    pFrame->n_inliers_ls++;
                }
            }
        }
    }

    }

    if (pFrame->n_inliers != (pFrame->n_inliers_pt + pFrame->n_inliers_ls))
        throw runtime_error("[StVO; stereoFrameHandler] Assetion failed: n_inliers != (n_inliers_pt + n_inliers_ls)");
}

bool Optimizer::PoseOptimizationWithLine(Frame *pFrame)
{

    // definitions
    Matrix4d DT, DT_;
    Matrix6d DT_cov;
    double   err = numeric_limits<double>::max();
    err = -1.0;

    if(pFrame->mTcw_prev.empty())
        pFrame->mTcw_prev = pFrame->mTcw;     // After Relocalization(), mTcw_prev is empty.

    // set init pose (depending on the use of prior information or not, and on the goodness of previous solution)
    if( Config::useMotionModel() )
    {
        DT     = Converter::toMatrix4d(pFrame->mTcw) * Converter::toInvMatrix4d(pFrame->mTcw_prev);
        // DT     = expmap_se3(logmap_se3( inverse_se3( pFrame->DT ) )); //pFrame->DT;
        DT_cov = pFrame->DT_cov;
        // double e_prev = pFrame->err_norm;
        // if( !isGoodSolution(DT,DT_cov,e_prev) )
        //     DT = Matrix4d::Identity();
    }
    else
        DT = Matrix4d::Identity();

    Matrix4d Tlw = Converter::toMatrix4d(pFrame->mTcw_prev);
    Matrix3d Rlw = Tlw.block<3,3>(0,0);
    Vector3d tlw = Tlw.col(3).head(3);

    const int NP = pFrame->mvpMapPoints.size();
    pFrame->mv3DpointInPrevFrame.resize(NP);
    for(int i=0; i<NP; ++i)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP && !pFrame->mvbOutlier[i])
        {
            cv::Mat Xw = pMP->GetWorldPos();
            Vector3d P_w(Xw.at<float>(0),Xw.at<float>(1),Xw.at<float>(2));
            Vector3d P_prevframe = Rlw*P_w+tlw;
            pFrame->mv3DpointInPrevFrame[i] = P_prevframe;
        }
    }

    const int NL = pFrame->mvpMapLines.size();
    pFrame->mv3DlineInPrevFrame.resize(NL);
    for(int i=0; i<NL; ++i)
    {
        MapLine* pML = pFrame->mvpMapLines[i];
        if(pML && !pFrame->mvbOutlier_Line[i])
        {
            Vector3d sP_prevframe = Rlw*pML->mWorldPos_sP+tlw;
            Vector3d eP_prevframe = Rlw*pML->mWorldPos_eP+tlw;
            pFrame->mv3DlineInPrevFrame[i] = make_pair(sP_prevframe, eP_prevframe);
        }
    }

    // optimization mode
    int mode = 0;   // GN - GNR - LM

    // solver
    if(true) {
        DT_ = DT;
        gaussNewtonOptimization(DT_,DT_cov,err,Config::maxItersRef(), pFrame);
        removeOutliers(DT_, pFrame);
        DT_ = DT;                                          // !!! DT, not DT_
        gaussNewtonOptimization(DT_,DT_cov,err,Config::maxItersRef(), pFrame);
        removeOutliers(DT_, pFrame);
//        DT_ = DT;
//        gaussNewtonOptimization(DT_,DT_cov,err,Config::maxItersRef(), pFrame);
//        removeOutliers(DT_, pFrame);
        DT_ = DT;
        gaussNewtonOptimizationRobust(DT_,DT_cov,err,Config::maxItersRef(), pFrame);
        removeOutliers(DT_, pFrame);
        DT_ = DT;
        gaussNewtonOptimization(DT_,DT_cov,err,Config::maxItersRef(), pFrame);
        removeOutliers(DT_, pFrame);

        DT = DT_;

        isGoodSolution(DT,DT_cov,err);
        pFrame->DT       =  expmap_se3(logmap_se3( inverse_se3( DT ) ));
        pFrame->DT_cov   = DT_cov;
        Matrix4d Twf = Converter::toInvMatrix4d(pFrame->mTcw_prev);         // T^w_lastframe
        Matrix4d Twc = expmap_se3(logmap_se3(Twf * pFrame->DT));                                    // T^w_currentframe = T^w_lastframe * T^last_current
        cv::Mat Tcw = Converter::toInvCvMat(Twc);
        pFrame->SetPose(Tcw);
        pFrame->err_norm = err;
//        pFrame->Tfw      = expmap_se3(logmap_se3( pFrame->Tfw * pFrame->DT ));
//        pFrame->Tfw_cov  = unccomp_se3( pFrame->Tfw, pFrame->Tfw_cov, DT_cov );
        SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
        pFrame->DT_cov_eig = eigensolver.eigenvalues();

        return true;
    }
    else {

    if( pFrame->n_inliers >= Config::minFeatures() )
    {
        // optimize
        DT_ = DT;
        if( mode == 0 ) gaussNewtonOptimization(DT_,DT_cov,err,Config::maxIters(), pFrame);
        else if( mode == 1 ) gaussNewtonOptimizationRobust(DT_,DT_cov,err,Config::maxIters(), pFrame);
        else if( mode == 2 ) levenbergMarquardtOptimization(DT_,DT_cov,err,Config::maxIters(), pFrame);

        // remove outliers (implement some logic based on the covariance's eigenvalues and optim error)
        if( isGoodSolution(DT_,DT_cov,err) )
        {
            removeOutliers(DT_, pFrame);
            // refine without outliers
            if( pFrame->n_inliers >= Config::minFeatures() )
            {
                if( mode == 0 )
                {
                    gaussNewtonOptimization(DT,DT_cov,err,Config::maxItersRef(), pFrame);
                    removeOutliers(DT, pFrame);                                             // !!! DT, not DT_
                    gaussNewtonOptimization(DT,DT_cov,err,Config::maxItersRef(), pFrame);
                    removeOutliers(DT, pFrame);
                    gaussNewtonOptimizationRobust(DT,DT_cov,err,Config::maxItersRef(), pFrame);
                    removeOutliers(DT, pFrame);
                    gaussNewtonOptimization(DT,DT_cov,err,Config::maxItersRef(), pFrame);
                }
                else if( mode == 1 ) gaussNewtonOptimizationRobust(DT,DT_cov,err,Config::maxItersRef(), pFrame);
                else if( mode == 2 ) levenbergMarquardtOptimization(DT,DT_cov,err,Config::maxItersRef(), pFrame);
            }
            else
            {
                DT     = Matrix4d::Identity();
                cout << "n_inliers:" << pFrame->n_inliers << ", not enough inliers for next optimization (after optimization&removal)" << endl;
            }
        }
        else
        {
            cout << "optimization didn't converge, perform optimizationRobust" << endl;
            gaussNewtonOptimizationRobust(DT,DT_cov,err,Config::maxItersRef(), pFrame);
        }
    }
    else
    {
        // DT     = Matrix4d::Identity(); // DT     = Converter::toMatrix4d(pFrame->mTcw);
        cout << "n_inliers:" << pFrame->n_inliers << ", not enough inliers (before optimization)" << endl;
    }

    // set estimated pose
    if( isGoodSolution(DT,DT_cov,err) && DT != Matrix4d::Identity() )
    {
        pFrame->DT       =  expmap_se3(logmap_se3( inverse_se3( DT ) ));
        pFrame->DT_cov   = DT_cov;
        Matrix4d Twf = Converter::toInvMatrix4d(pFrame->mTcw_prev);         // T^w_lastframe
        Matrix4d Twc = expmap_se3(logmap_se3(Twf * pFrame->DT));                                    // T^w_currentframe = T^w_lastframe * T^last_current
        cv::Mat Tcw = Converter::toInvCvMat(Twc);
        pFrame->SetPose(Tcw);
        pFrame->err_norm = err;
//        pFrame->Tfw      = expmap_se3(logmap_se3( pFrame->Tfw * pFrame->DT ));
//        pFrame->Tfw_cov  = unccomp_se3( pFrame->Tfw, pFrame->Tfw_cov, DT_cov );
        SelfAdjointEigenSolver<Matrix6d> eigensolver(DT_cov);
        pFrame->DT_cov_eig = eigensolver.eigenvalues();
        return true;
    }
    else
    {
        cout<<"Can not estimate current frame pose, set Identity!"<<endl;
        //setAsOutliers();
        pFrame->DT = Matrix4d::Identity();
        // pFrame->SetPose(pFrame->mTcw_prev);
        pFrame->DT_cov   = Matrix6d::Zero();
        pFrame->err_norm = -1.0;
//        pFrame->Tfw      = pFrame->Tfw;
//        pFrame->Tfw_cov  = pFrame->Tfw_cov;
        pFrame->DT_cov_eig = Vector6d::Zero();
        return false;
    }

    }
}

void Optimizer::gaussNewtonOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters, Frame* pFrame)
{
    Matrix6d H;
    Vector6d g, DT_inc;
    double err, err_prev = 999999999.9;

    int iters;
    for( iters = 0; iters < max_iters; iters++)
    {
        // estimate hessian and gradient (select)
        optimizeFunctions( DT, H, g, err, pFrame);
        if (err > err_prev) {
            if (iters > 0)
                break;
            err_ = -1.0;
            return;
        }
        // if the difference is very small stop
        if( ( ( err < Config::minError()) || abs(err-err_prev) < Config::minErrorChange() ) ) {
            // cout << "[StVO] Small optimization improvement" << endl;
            break;
        }
        // update step
        ColPivHouseholderQR<Matrix6d> solver(H);
        DT_inc = solver.solve(g);
        DT  << DT * inverse_se3( expmap_se3(DT_inc) );
        // if the parameter change is small stop
        if( DT_inc.head(3).norm() < Config::minErrorChange() && DT_inc.tail(3).norm() < Config::minErrorChange()) {
            // cout << "[StVO] Small optimization solution variance" << endl;
            break;
        }
        // update previous values
        err_prev = err;
    }

    DT_cov = H.inverse();  //DT_cov = Matrix6d::Identity();
    err_   = err;
}

void Optimizer::gaussNewtonOptimizationRobust(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters, Frame* pFrame)
{

    Matrix4d DT_;
    Matrix6d H;
    Vector6d g, DT_inc;
    double err, err_prev = 999999999.9;
    bool solution_is_good = true;
    DT_ = DT;

    // plot initial solution
    int iters;
    for( iters = 0; iters < max_iters; iters++)
    {
        // estimate hessian and gradient (select)
        optimizeFunctionsRobust( DT, H, g, err, pFrame);
        // if the difference is very small stop
        if( ( fabs(err-err_prev) < Config::minErrorChange() ) || ( err < Config::minError()) )// || err > err_prev )
            break;
        // update step
        ColPivHouseholderQR<Matrix6d> solver(H);
        DT_inc = solver.solve(g);
        if( solver.logAbsDeterminant() < 0.0 || solver.info() != Success )
        {
            solution_is_good = false;
            break;
        }
        DT  << DT * inverse_se3( expmap_se3(DT_inc) );
        // if the parameter change is small stop (TODO: change with two parameters, one for R and another one for t)
        if( DT_inc.norm() < Config::minErrorChange() )
            break;
        // update previous values
        err_prev = err;
    }

    if( solution_is_good )
    {
        DT_cov = H.inverse();  //DT_cov = Matrix6d::Identity();
        err_   = err;
    }
    else
    {
        DT = DT_;
        err_ = -1.0;
        DT_cov = Matrix6d::Identity();
    }

}

void Optimizer::levenbergMarquardtOptimization(Matrix4d &DT, Matrix6d &DT_cov, double &err_, int max_iters, Frame* pFrame)
{
    Matrix6d H;
    Vector6d g, DT_inc;
    double err, err_prev = 999999999.9;

    double lambda   = 0.000000001;
    double lambda_k = 4.0;

    // form the first iteration
    //    optimizeFunctionsRobust( DT, H, g, err, pFrame);
    optimizeFunctions( DT, H, g, err, pFrame);

    // initial guess of lambda
    double Hmax = 0.0;
    for( int i = 0; i < 6; i++)
    {
        if( H(i,i) > Hmax || H(i,i) < -Hmax )
            Hmax = fabs( H(i,i) );
    }
    lambda *= Hmax;

    // solve the first iteration
    for(int i = 0; i < 6; i++)
        H(i,i) += lambda;// * H(i,i) ;
    ColPivHouseholderQR<Matrix6d> solver(H);
    DT_inc = solver.solve(g);
    DT  << DT * inverse_se3( expmap_se3(DT_inc) );
    err_prev = err;

    // start Levenberg-Marquardt minimization
    //plotStereoFrameProjerr(DT,0);
    for( int iters = 1; iters < max_iters; iters++)
    {
        // estimate hessian and gradient (select)
    //        optimizeFunctionsRobust( DT, H, g, err, pFrame);
        optimizeFunctions( DT, H, g, err, pFrame);
        // if the difference is very small stop
        if( ( fabs(err-err_prev) < Config::minErrorChange() ) || ( err < Config::minError()) )
            break;
        // add lambda to hessian
        for(int i = 0; i < 6; i++)
            H(i,i) += lambda;// * H(i,i) ;
        // update step
        ColPivHouseholderQR<Matrix6d> solver(H);
        DT_inc = solver.solve(g);
        // update lambda
        if( err > err_prev )
            lambda /= lambda_k;
        else
        {
            lambda *= lambda_k;
            DT  << DT * inverse_se3( expmap_se3(DT_inc) );
        }
        // plot each iteration
        //plotStereoFrameProjerr(DT,iters+1);
        // if the parameter change is small stop
        if( DT_inc.head(3).norm() < Config::minErrorChange() && DT_inc.tail(3).norm() < Config::minErrorChange())
            break;
        // update previous values
        err_prev = err;
    }

    DT_cov = H.inverse();  //DT_cov = Matrix6d::Identity();
    err_   = err;
}

void Optimizer::optimizeFunctions(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e, Frame* pFrame)
{

    // define hessians, gradients, and residuals
    Matrix6d H_l, H_p;
    Vector6d g_l, g_p;
    double   e_l = 0.0, e_p = 0.0;
    // double S_l, S_p;
    H   = Matrix6d::Zero(); H_l = H; H_p = H;
    g   = Vector6d::Zero(); g_l = g; g_p = g;
    e   = 0.0;

    // point features
    int N_p = 0;        // orbslam number of point features
    int N_l = 0;
    Matrix3d DR = DT.block<3,3>(0,0);
    Vector3d Dt = DT.col(3).head(3);

    {
    unique_lock<mutex> lockP(MapPoint::mGlobalMutex);
    unique_lock<mutex> lockL(MapLine::mGlobalMutex);
    // from matched_pt to mvpMapPoints
    const int NP = pFrame->mvpMapPoints.size();
    for(int i=0; i<NP; ++i)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP && !pFrame->mvbOutlier[i])
        {
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            Vector2d pl_obs;
            pl_obs << kpUn.pt.x, kpUn.pt.y;

            Vector3d P_lastframe = pFrame->mv3DpointInPrevFrame[i];
            Vector3d P_ = DR*P_lastframe+Dt;
            Vector2d pl_proj = pFrame->projection( P_ );

            // projection error
            Vector2d err_i    = pl_proj - pl_obs;
            double err_i_norm = err_i.norm();
            // estimate variables for J, H, and g
            double gx   = P_(0);
            double gy   = P_(1);
            double gz   = P_(2);
            double gz2  = gz*gz;
            double fgz2 = pFrame->fx / std::max(Config::homogTh(),gz2);
            double dx   = err_i(0);
            double dy   = err_i(1);
            // jacobian
            Vector6d J_aux;
            J_aux << + fgz2 * dx * gz,
                     + fgz2 * dy * gz,
                     - fgz2 * ( gx*dx + gy*dy ),
                     - fgz2 * ( gx*gy*dx + gy*gy*dy + gz*gz*dy ),
                     + fgz2 * ( gx*gx*dx + gz*gz*dx + gx*gy*dy ),
                     + fgz2 * ( gx*gz*dy - gy*gz*dx );
            J_aux = J_aux / std::max(Config::homogTh(),err_i_norm);
            // define the residue
            double r = err_i_norm * sqrt(pFrame->mvInvLevelSigma2[kpUn.octave]);
            // if employing robust cost function
            double w  = 1.0;
            w = robustWeightCauchy(r);

            // if down-weighting far samples
            //double zdist   = P_(2) * 1.0 / ( cam->getB()*cam->getFx());
            //if( false ) w *= 1.0 / ( s2 + zdist * zdist );

            // update hessian, gradient, and error
            H_p += J_aux * J_aux.transpose() * w;
            g_p += J_aux * r * w;
            e_p += r * r * w;
            N_p++;
        }
    }

    // line segment features
    const int NL = pFrame->mvpMapLines.size();
    for(int i=0; i<NL; ++i)
    {
        MapLine* pML = pFrame->mvpMapLines[i];
        if(pML && !pFrame->mvbOutlier_Line[i])
        {
            //LineFeature* obs = pFrame->stereo_ls[i];
            Vector3d l_obs = pFrame->mvle_l[i]; //obs->le_obs;

            Vector3d sP_lastframe = pFrame->mv3DlineInPrevFrame[i].first;
            Vector3d sP_ = DR*sP_lastframe+Dt;
            Vector3d eP_lastframe = pFrame->mv3DlineInPrevFrame[i].second;
            Vector3d eP_ = DR*eP_lastframe+Dt;
            Vector2d spl_proj = pFrame->projection( sP_ );
            Vector2d epl_proj = pFrame->projection( eP_ );

            // projection error
            Vector2d err_i;
            err_i(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_i(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            double err_i_norm = err_i.norm();
            // estimate variables for J, H, and g
            // -- start point
            double gx   = sP_(0);
            double gy   = sP_(1);
            double gz   = sP_(2);
            double gz2  = gz*gz;
            double fgz2 = pFrame->fx / std::max(Config::homogTh(),gz2);
            double ds   = err_i(0);
            double de   = err_i(1);
            double lx   = l_obs(0);
            double ly   = l_obs(1);
            Vector6d Js_aux;
            Js_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );
            // -- end point
            gx   = eP_(0);
            gy   = eP_(1);
            gz   = eP_(2);
            gz2  = gz*gz;
            fgz2 = pFrame->fx / std::max(Config::homogTh(),gz2);
            Vector6d Je_aux, J_aux;
            Je_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );
            // jacobian
            J_aux = ( Js_aux * ds + Je_aux * de ) / std::max(Config::homogTh(),err_i_norm);

            // define the residue
            double r = err_i_norm * sqrt(pFrame->mvInvLevelSigma2[pFrame->mvKeysUn_Line[i].octave]);

            // if employing robust cost function
            double w  = 1.0;
            w = robustWeightCauchy(r) ;

            // estimating overlap between line segments
            bool has_overlap = true;
            Vector2d spl_obs = Vector2d(pFrame->mvKeysUn_Line[i].startPointX,pFrame->mvKeysUn_Line[i].startPointY);
            Vector2d epl_obs = Vector2d(pFrame->mvKeysUn_Line[i].endPointX,pFrame->mvKeysUn_Line[i].endPointY);
            double overlap = pFrame->lineSegmentOverlap( spl_obs, epl_obs, spl_proj, epl_proj );
            if( has_overlap )
                w *= overlap;

            // if down-weighting far samples
            /*double zdist = max( sP_(2), eP_(2) ) / ( cam->getB()*cam->getFx());
            if( false )
                w *= 1.0 / ( s2 + zdist * zdist );*/

            // update hessian, gradient, and error
            H_l += J_aux * J_aux.transpose() * w;
            g_l += J_aux * r * w;
            e_l += r * r * w;
            N_l++;
        }
    }
    }

    // sum H, g and err from both points and lines
    H = H_p + H_l;
    g = g_p + g_l;
    e = e_p + e_l;

    // normalize error
    e /= (N_l+N_p);

}

void Optimizer::optimizeFunctionsRobust(Matrix4d DT, Matrix6d &H, Vector6d &g, double &e, Frame* pFrame)
{

    // define hessians, gradients, and residuals
    Matrix6d H_l, H_p;
    Vector6d g_l, g_p;
    double e_l = 0.0, e_p = 0.0;
    // double S_l, S_p;
    H   = Matrix6d::Zero(); H_l = H; H_p = H;
    g   = Vector6d::Zero(); g_l = g; g_p = g;
    e   = 0.0;

    vector<double> res_p, res_l;

    Matrix3d DR = DT.block<3,3>(0,0);
    Vector3d Dt = DT.col(3).head(3);

    {
    unique_lock<mutex> lockP(MapPoint::mGlobalMutex);
    unique_lock<mutex> lockL(MapLine::mGlobalMutex);
    // point features pre-weight computation
    const int NP = pFrame->mvpMapPoints.size();
    for(int i=0; i<NP; ++i)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP && !pFrame->mvbOutlier[i])
        {
            Vector3d P_lastframe = pFrame->mv3DpointInPrevFrame[i];
            Vector3d P_ = DR*P_lastframe+Dt;
            Vector2d proj = pFrame->projection( P_ );

            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            Vector2d obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            // projection error
            Vector2d err_i    = proj - obs;
            res_p.push_back( err_i.norm());
        }
    }

    // line segment features pre-weight computation
    const int NL = pFrame->mvpMapLines.size();
    for(int i=0; i<NL; ++i)
    {
        MapLine* pML = pFrame->mvpMapLines[i];
        if(pML && !pFrame->mvbOutlier_Line[i])
        {
            //LineFeature* obs = pFrame->stereo_ls[i];
            Vector3d l_obs = pFrame->mvle_l[i]; //obs->le_obs;

            Vector3d sP_lastframe = pFrame->mv3DlineInPrevFrame[i].first;
            Vector3d sP_ = DR*sP_lastframe+Dt;
            Vector3d eP_lastframe = pFrame->mv3DlineInPrevFrame[i].second;
            Vector3d eP_ = DR*eP_lastframe+Dt;

            Vector2d spl_proj = pFrame->projection( sP_ );
            Vector2d epl_proj = pFrame->projection( eP_ );

            // projection error
            Vector2d err_i;
            err_i(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_i(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            res_l.push_back( err_i.norm() );
        }
    }
    }
    // estimate scale of the residuals
    double s_p = 1.0, s_l = 1.0;
    double th_min = 0.0001;
    double th_max = sqrt(7.815);

    if( false )
    {

        res_p.insert( res_p.end(), res_l.begin(), res_l.end() );
        s_p = vector_stdv_mad( res_p );

        //cout << s_p << "\t";
        //if( res_p.size() < 4*Config::minFeatures() )
            //s_p = 1.0;
        //cout << s_p << endl;

        if( s_p < th_min )
            s_p = th_min;
        if( s_p > th_max )
            s_p = th_max;

        s_l = s_p;

    }
    else
    {

        s_p = vector_stdv_mad( res_p );
        s_l = vector_stdv_mad( res_l );

        if( s_p < th_min )
            s_p = th_min;
        if( s_p > th_max )
            s_p = th_max;

        if( s_l < th_min )
            s_l = th_min;
        if( s_l > th_max )
            s_l = th_max;
    }

    // point features
    int N_p = 0;
    int N_l = 0;

    {
    unique_lock<mutex> lockP(MapPoint::mGlobalMutex);
    unique_lock<mutex> lockL(MapLine::mGlobalMutex);
    const int NP = pFrame->mvpMapPoints.size();
    for(int i=0; i<NP; ++i)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP && !pFrame->mvbOutlier[i])
        {
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            Vector2d pl_obs;
            pl_obs << kpUn.pt.x, kpUn.pt.y;

            Vector3d P_lastframe = pFrame->mv3DpointInPrevFrame[i];
            Vector3d P_ = DR*P_lastframe+Dt;
            Vector2d pl_proj = pFrame->projection( P_ );

            // projection error
            Vector2d err_i    = pl_proj - pl_obs;
            double err_i_norm = err_i.norm();
            // estimate variables for J, H, and g
            double gx   = P_(0);
            double gy   = P_(1);
            double gz   = P_(2);
            double gz2  = gz*gz;
            double fgz2 = pFrame->fx / std::max(Config::homogTh(),gz2);
            double dx   = err_i(0);
            double dy   = err_i(1);
            // jacobian
            Vector6d J_aux;
            J_aux << + fgz2 * dx * gz,
                     + fgz2 * dy * gz,
                     - fgz2 * ( gx*dx + gy*dy ),
                     - fgz2 * ( gx*gy*dx + gy*gy*dy + gz*gz*dy ),
                     + fgz2 * ( gx*gx*dx + gz*gz*dx + gx*gy*dy ),
                     + fgz2 * ( gx*gz*dy - gy*gz*dx );
            J_aux = J_aux / std::max(Config::homogTh(),err_i_norm);
            // define the residue
            double r = err_i_norm;
            // double r = err_i_norm * sqrt(pFrame->mvInvLevelSigma2[kpUn.octave]);
            // if employing robust cost function
            double w  = 1.0;
            double x = r / s_p;
            w = robustWeightCauchy(x) ;

            // if using uncertainty weights
            //----------------------------------------------------
            // if( false ) {}
            /*
            {
            
                Matrix2d covp;
                Matrix3d covP_ = pMP->covP_an;
                MatrixXd J_Pp(2,3), J_pr(1,2);
                // uncertainty of the projection
                J_Pp  << gz, 0.f, -gx, 0.f, gz, -gy;
                J_Pp  << J_Pp * DT.block(0,0,3,3);
                covp  << J_Pp * covP_ * J_Pp.transpose();
                covp  << covp / std::max(Config::homogTh(),gz2*gz2);
                // Covariance of the 3D projection \hat{p} up to f2*b2*sigma2

                covp  = sqrt(s2) * cam->getB()* cam->getFx() * covp;
                covp(0,0) += s2;
                covp(1,1) += s2;
                // Point Uncertainty constants
                    // bsigmaP   = f * baseline * sigmaP;
                    // bsigmaP   = bsigmaP * bsigmaP;
                    // bsigmaP_inv   = 1.f / bsigmaP;
                    // sigmaP2       = sigmaP * sigmaP;
                    // sigmaP2_inv   = 1.f / sigmaP2;
                    // // Line Uncertainty constants
                    // bsigmaL   = baseline * sigmaL;
                    // bsigmaL   = bsigmaL * bsigmaL;
                    // bsigmaL_inv   = 1.f / bsigmaL;
                // uncertainty of the residual
                J_pr << dx / r, dy / r;
                double cov_res = (J_pr * covp * J_pr.transpose())(0);
                cov_res = 1.0 / std::max(Config::homogTh(),cov_res);
                double zdist   = P_(2) * 1.0 / ( cam->getB()*cam->getFx());

                //zdist   = 1.0 / std::max(Config::homogTh(),zdist);
                    // cout.setf(ios::fixed,ios::floatfield); cout.precision(8);
                    // cout << endl << cov_res << " " << 1.0 / cov_res << " " << zdist << " " << 1.0 / zdist << " " << 1.0 / (zdist*40.0) << "\t"
                    //      << 1.0 / ( 1.0 +  cov_res * cov_res + zdist * zdist ) << " \t"
                    //      << 1.0 / ( cov_res * cov_res + zdist * zdist )
                    //      << endl;
                    // cout.setf(ios::fixed,ios::floatfield); cout.precision(3);
                //w *= 1.0 / ( cov_res * cov_res + zdist * zdist );
                w *= 1.0 / ( s2 + zdist * zdist );
            
            }
            */

            //----------------------------------------------------

            // update hessian, gradient, and error
            H_p += J_aux * J_aux.transpose() * w;
            g_p += J_aux * r * w;
            e_p += r * r * w;
            N_p++;
        }
    }

    const int NL = pFrame->mvpMapLines.size();
    // line segment features
    for(int i=0; i<NL; ++i)
    {
        MapLine* pML = pFrame->mvpMapLines[i];
        if(pML && !pFrame->mvbOutlier_Line[i])
        {
            //LineFeature* obs = pFrame->stereo_ls[i];
            Vector3d l_obs = pFrame->mvle_l[i]; //obs->le_obs;

            Vector3d sP_lastframe = pFrame->mv3DlineInPrevFrame[i].first;
            Vector3d sP_ = DR*sP_lastframe+Dt;
            Vector3d eP_lastframe = pFrame->mv3DlineInPrevFrame[i].second;
            Vector3d eP_ = DR*eP_lastframe+Dt;

            Vector2d spl_proj = pFrame->projection( sP_ );
            Vector2d epl_proj = pFrame->projection( eP_ );

            // projection error
            Vector2d err_i;
            err_i(0) = l_obs(0) * spl_proj(0) + l_obs(1) * spl_proj(1) + l_obs(2);
            err_i(1) = l_obs(0) * epl_proj(0) + l_obs(1) * epl_proj(1) + l_obs(2);
            double err_i_norm = err_i.norm();
            // estimate variables for J, H, and g
            // -- start point
            double gx   = sP_(0);
            double gy   = sP_(1);
            double gz   = sP_(2);
            double gz2  = gz*gz;
            double fgz2 = pFrame->fx / std::max(Config::homogTh(),gz2);
            double ds   = err_i(0);
            double de   = err_i(1);
            double lx   = l_obs(0);
            double ly   = l_obs(1);
            Vector6d Js_aux;
            Js_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );
            // -- end point
            gx   = eP_(0);
            gy   = eP_(1);
            gz   = eP_(2);
            gz2  = gz*gz;
            fgz2 = pFrame->fx / std::max(Config::homogTh(),gz2);
            Vector6d Je_aux, J_aux;
            Je_aux << + fgz2 * lx * gz,
                      + fgz2 * ly * gz,
                      - fgz2 * ( gx*lx + gy*ly ),
                      - fgz2 * ( gx*gy*lx + gy*gy*ly + gz*gz*ly ),
                      + fgz2 * ( gx*gx*lx + gz*gz*lx + gx*gy*ly ),
                      + fgz2 * ( gx*gz*ly - gy*gz*lx );
            // jacobian
            J_aux = ( Js_aux * ds + Je_aux * de ) / std::max(Config::homogTh(),err_i_norm);
            // define the residue
            // double sqrt_s2 = sqrt(pFrame->mvInvLevelSigma2[pFrame->mvKeysUn_Line[i].octave])
            double r = err_i_norm;
            // if employing robust cost function
            double w  = 1.0;
            double x = r / s_l;
            w = robustWeightCauchy(x) ;
            // estimating overlap between line segments
            bool has_overlap = true;
            Vector2d spl_obs = Vector2d(pFrame->mvKeysUn_Line[i].startPointX,pFrame->mvKeysUn_Line[i].startPointY);
            Vector2d epl_obs = Vector2d(pFrame->mvKeysUn_Line[i].endPointX,pFrame->mvKeysUn_Line[i].endPointY);
            double overlap = pFrame->lineSegmentOverlap( spl_obs, epl_obs, spl_proj, epl_proj );
            if( has_overlap )
                w *= overlap;

            //----------------- DEBUG: 27/11/2017 ----------------------
            if( false )
            {
                /*
                //double cov3d = (*it)->cov3d;
                //cov3d = 1.0;
                //w *= 1.0 / ( 1.0 +  cov3d * cov3d + zdist * zdist );
                double zdist = max( sP_(2), eP_(2) ) / ( cam->getB()*cam->getFx());
                w *= 1.0 / ( s2 + zdist * zdist );
                */
            }
            //----------------------------------------------------------

            // update hessian, gradient, and error
            H_l += J_aux * J_aux.transpose() * w;
            g_l += J_aux * r * w;
            e_l += r * r * w;
            N_l++;
        }
    }
    }
    // sum H, g and err from both points and lines
    H = H_p + H_l;
    g = g_p + g_l;
    e = e_p + e_l;

    // normalize error
    e /= (N_l+N_p);

}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {                
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}


void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}


} //namespace ORB_SLAM
