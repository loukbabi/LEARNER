#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"sophus/sim3.hpp"
#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"
#include"Matchers/lightglue_onnx.h"

namespace ORB_SLAM3
{
    struct SuperGlueConfig {
        int image_width;
        int image_height;
        int dla_core;
        std::vector<std::string> input_tensor_names;
        std::vector<std::string> output_tensor_names;
        std::string onnx_file;
        std::string engine_file;
    };

    class SPmatcher
    {
    public:

        SPmatcher(float thre);
        int SearchBySP(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
        int SearchBySP(Frame &F, const std::vector<MapPoint*> &vpMapPoints);
        int SearchBySP(Frame &CurrentFrame, Frame &LastFrame);
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3f &Scw, const vector<MapPoint*> &vpPoints,const std::vector<KeyFrame*> &vpPointsKFs,
                                       vector<MapPoint*> &vpMatched, int th, float ratioHamming);
        static float DescriptorDistance_sp(const cv::Mat &a, const cv::Mat &b);
        void ConvertMatchesToVector(const std::vector<cv::DMatch>& matches, std::vector<int>& vnMatches12);

        int MatchingPoints(const Eigen::Matrix<double, 259, Eigen::Dynamic>& features0, 
            const Eigen::Matrix<double, 259, Eigen::Dynamic>& features1, std::vector<cv::DMatch>& matches,  bool outlier_rejection=false);
        int MatchingPoints_onnx(Frame &f1, Frame &f2,vector<int> &vnMatches12);
        int MatchingPoints(Frame &f1, Frame &f2, vector<cv::Point2f> &vbPrevMatched,vector<int> &vnMatches12, bool outlier_rejection=false);
        int MatchingPoints_onnx(std::vector<cv::KeyPoint> kpts0, const std::vector<cv::KeyPoint> kpts1, cv::Mat desc0,const cv::Mat desc1, std::vector<int>& vnMatches12);
        int MatchingPoints_onnx(std::vector<cv::Point2f> kpts0, std::vector<cv::Point2f> kpts1, float* desc0, float* desc1);
        int MatchingPoints_onnx(std::vector<cv::Point2f> kpts0, std::vector<cv::Point2f> kpts1, cv::Mat desc0, cv::Mat desc1, std::vector<int>& vnMatches12);
        Eigen::Matrix<double, 259, Eigen::Dynamic> NormalizeKeypoints(
            const Eigen::Matrix<double, 259, Eigen::Dynamic> &features, int width, int height
        );
        void plotspmatch(cv::Mat frame1, cv::Mat frame2,std::vector<cv::KeyPoint> kpts1, std::vector<cv::KeyPoint> kpts2,  std::vector<int> vmatches12);
        // SPmatcher(float nnratio=0.6, bool checkOri=true);

        // Computes the Hamming distance between two ORB descriptors
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
        // Used to track the local map (Tracking)
        int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

        
        int SearchByBoWSP(KeyFrame* pKF, Frame &F, vector<MapPoint*> &vpMapPointMatches);
        // Project MapPoints tracked in last frame into the current frame and search matches.
        // Used to track from previous frame (Tracking)
        int SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const float th, const bool bMono);
        
        // Project MapPoints seen in KeyFrame into the Frame and search matches.
        // Used in relocalisation (Tracking)
        int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in loop detection (Loop Closing)
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in Place Recognition (Loop Closing and Merging)
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);

        // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
        // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
        // Used in Relocalisation and Loop Detection
        int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
        int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12,vector<cv::DMatch>& vmatches);
        int SearchByProjection1(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th, const bool bFarPoints, const float thFarPoints);
        int SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const int th);
        // Matching for the Map Initialization (only used in the monocular case)
        int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12);
        int SearchByBoWSP(KeyFrame* pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12, vector<cv::DMatch> & vmatches);
        Eigen::Matrix<double, 259, Eigen::Dynamic> ConvertToEigenMatrix(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& descriptors);

        // Matching to triangulate new MapPoints. Check Epipolar Constraint.
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2,
                                   std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

        // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
        // In the stereo and RGB-D case, s12=1
        // int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
        int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th);

        // Project MapPoints into KeyFrame and search for duplicated MapPoints.
        int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0, const bool bRight = false);
        int Fuse_onnx(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th, const bool bRight = false);
        // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
        int Fuse(KeyFrame* pKF, Sophus::Sim3f &Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

    public:

        static const float TH_LOW;
        static const float TH_HIGH;
        static const int HISTO_LENGTH;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LightGlueDecoupleOnnxRunner* featureMatcher;
    protected:
        float RadiusByViewingCos(const float &viewCos);

        void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

        float mfNNratio;
        bool mbCheckOrientation;
    
    private:
        // SuperGlue superglue;
        // SuperGlueConfig _superglue_config;
        
    };

    typedef std::shared_ptr<SPmatcher> SPmatcherPtr;

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
