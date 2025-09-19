#pragma once

#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

struct Detection {
    int classId;
    float confidence;
    cv::Rect box;
    std::string label;  
};

class YoloV8Detector {
public:
    YoloV8Detector(const std::string& modelPath, float confThreshold = 0.48f, float nmsThreshold = 0.5f);
    ~YoloV8Detector();
    
    std::vector<Detection> detect(const cv::Mat& image);
    
    


private:
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
std::string inputName_;
std::string outputName_;

    std::vector<int64_t> inputDims_;
    std::vector<int64_t> outputDims_;
    float confThreshold_;
    float nmsThreshold_;
};


