#include "Matchers/Configuration.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "Extractors/SPextractor.h"
#include "ORBextractor.h"
// #include "SuperPoint.h"
#include <opencv2/core/eigen.hpp>

#include "Extractors/super_point.h"
#include <utility>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "YoloDetection.h"
using namespace cv;
using namespace std;

namespace ORB_SLAM3
{
enum KpState {
    DYNAMIC = 0,
    CAN_MOVE = 1,
    STATIONARY = 2
};
const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;


const float factorPI = (float)(CV_PI/180.f);

SPextractor::SPextractor(int _nfeatures, float _scaleFactor, int _nlevels,
         float _iniThFAST, float _minThFAST):
    nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
    iniThFAST(_iniThFAST), minThFAST(_minThFAST)
{
   
    if(mModelstr == "onnx")
    {   Configuration cfg;
        cfg.device = "cuda";
        cfg.extractorPath = "/home/anast/Workspaces/PhD_ws/src/SLAM_Methods/Rover-SLAM/onnxmodel/superpoint.onnx";
        cfg.extractorType = "superpoint";
        featureExtractor = new SuperPointOnnxRunner();
        featureExtractor->InitOrtEnv(cfg);
        yolo = new YoloV8Detector("/home/anast/Workspaces/PhD_ws/src/SLAM_Methods/Rover-SLAM/onnxmodel/doorshumans.onnx");
    }
    

    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }
    for(int i = 0; i < mvLevelSigma2.size(); i++)
    {
        cout<<mvLevelSigma2[i]<<" assd";
    }
    cout<<endl;
    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mvImagePyramid.resize(nlevels);

    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

}

int SPextractor::operator()( InputArray _image,  vector<KeyPoint>& _keypoints,
                      cv::Mat& _descriptors)
{
    if(_image.empty())
        return 0;

    Mat image = _image.getMat();
    
    //cout << typeToString(image.type());
    assert(image.type() == CV_8UC1 );

    Mat descriptors;
    //两种模式，一种单层图像提取特征，一种多层金字塔提取特征
    int res = -1;
    if (nlevels == 1) 
        res = ExtractSingleLayer(image, _keypoints, _descriptors);
    else{
        ComputePyramid(image);
        res = ExtractMultiLayers(image, _keypoints, descriptors);
    }

    return res;
}

int SPextractor::ExtractSingleLayer(const cv::Mat &image, std::vector<cv::KeyPoint>& vKeyPoints, cv::Mat &Descriptors)
{
    if(mModelstr == "onnx"){
        Configuration cfg;
        cv::Mat image_copy = image.clone();
        cv::Mat inputImage = NormalizeImage(image_copy);
        featureExtractor->lastmatch = lastmatchnum;
        featureExtractor->Extractor_Inference(cfg , inputImage);
        featureExtractor->Extractor_PostProcess(cfg , std::move(featureExtractor->extractor_outputtensors[0]),vKeyPoints,Descriptors);
        FilterDynamicKeypoints(image, vKeyPoints, Descriptors);
    }
    else{

    }


    return vKeyPoints.size();
}

int SPextractor::ExtractMultiLayers(const cv::Mat &image, std::vector<cv::KeyPoint>& vKeyPoints, cv::Mat &Descriptors)
{
    ComputePyramid(image);
    int nKeyPoints = 0;
    vector<vector<cv::KeyPoint>> allKeypoints(nlevels);
    vector<cv::Mat> allDescriptors(nlevels);
    for (int level = 0; level < nlevels; ++level)
    {
        if(level == 0)
        {

        }
        else
        {

        }
        nKeyPoints +=allKeypoints[level].size();
    }
    vKeyPoints.clear();
    vKeyPoints.reserve(nKeyPoints);
    for (int level = 0; level < nlevels; ++level)
    {
        for(auto keypoint : allKeypoints[level])
        {
            keypoint.octave = level;
            keypoint.pt *= mvScaleFactor[level];
            vKeyPoints.emplace_back(keypoint);
        }
    }
    cv::vconcat(allDescriptors.data(), allDescriptors.size(), Descriptors);
    
    return vKeyPoints.size();
}

void SPextractor::FilterDynamicKeypoints(const cv::Mat& image,
                                         std::vector<cv::KeyPoint>& keypoints,
                                         cv::Mat& descriptors)
{
if (!yolo)
    return;

std::vector<Detection> detections = yolo->detect(image);
if (mpTracking) {
    mpTracking->SetYoloDetections(detections);  // Send to Tracking
}




std::vector<cv::KeyPoint> filtered;
cv::Mat filteredDesc;

 for (size_t i = 0; i < keypoints.size(); ++i)
    {
        int state = STATIONARY;  // default

        for (const auto& det : detections)
        {
            if (det.box.contains(keypoints[i].pt))
            {
                if (det.label == "person")
                    state = DYNAMIC;
                else if (det.label == "closed_door")
                    state = CAN_MOVE;

                break; // found one match is enough
            }
        }

        keypoints[i].class_id = state;

        if (state != DYNAMIC)
        {
            filtered.push_back(keypoints[i]);
            filteredDesc.push_back(descriptors.row(i));
        }
    }

    keypoints.swap(filtered);
    descriptors = filteredDesc.clone();
}
void SPextractor::SetTrackingPtr(ORB_SLAM3::Tracking* pTracker)
{
    mpTracking = pTracker;
}

void SPextractor::ComputePyramid(cv::Mat image)
{
    std::cout<<" compute pyramid !"<<std::endl;
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        Mat temp(wholeSize, image.type()), masktemp;
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101+BORDER_ISOLATED);            
        }
        else
        {
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101);            
        }
    }

}

} //namespace ORB_SLAM
