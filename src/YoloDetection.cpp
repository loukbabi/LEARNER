#include "YoloDetection.h" // Include your header

#include <iostream>
#include <algorithm>
#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
static const std::vector<std::string> coco_class_names = {
    "person",        // classId 0
    "closed_door"    // classId 1
};

YoloV8Detector::YoloV8Detector(const std::string& modelPath, float confThreshold, float nmsThreshold)
    : confThreshold_(confThreshold), nmsThreshold_(nmsThreshold) {

    env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YoloV8");

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

#ifdef USE_CUDA
    OrtCUDAProviderOptions cudaOptions;
    cudaOptions.device_id = 0;
    sessionOptions.AppendExecutionProvider_CUDA(cudaOptions);
#endif

    Ort::AllocatorWithDefaultOptions allocator;
    session_ = std::make_unique<Ort::Session>(env_, modelPath.c_str(), sessionOptions);

    Ort::AllocatedStringPtr input_name_ptr = session_->GetInputNameAllocated(0, allocator);
    Ort::AllocatedStringPtr output_name_ptr = session_->GetOutputNameAllocated(0, allocator);
inputName_ = std::string(input_name_ptr.get());
outputName_ = std::string(output_name_ptr.get());


    Ort::TypeInfo inputTypeInfo = session_->GetInputTypeInfo(0);
    auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
    inputDims_ = inputTensorInfo.GetShape();

    if (inputDims_[2] == -1 || inputDims_[3] == -1) {
        inputDims_[2] = 640;
        inputDims_[3] = 640;
    }

    Ort::TypeInfo outputTypeInfo = session_->GetOutputTypeInfo(0);
    auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
    outputDims_ = outputTensorInfo.GetShape();
}


    
YoloV8Detector::~YoloV8Detector() {

}

    
    // Perform object detection on a BGR image. Returns detections (filtering to persons class 0).

std::vector<Detection> YoloV8Detector::detect(const cv::Mat& imageBGR)
{

// ---------------------------------------------------------------------------
// run-time constants read from the ONNX input tensor (480 × 360)
// ---------------------------------------------------------------------------
const int netH = static_cast<int>(inputDims_[2]);   // 360
const int netW = static_cast<int>(inputDims_[3]);   // 480

// remember the native sequence size only once
static int seqW = 0, seqH = 0;
if (seqW == 0 || seqH == 0) {
    seqW = imageBGR.cols;
    seqH = imageBGR.rows;
    std::cout << "[YOLO] sequence size  = " << seqW << 'x' << seqH << '\n';
}

// ─────────────────── 1. resize (uniform stretch) ───────────────────
cv::Mat resized;
cv::resize(imageBGR, resized, cv::Size(netW, netH));      // 480×360

// ─────────────────── 2. ensure 3-channel RGB ───────────────────────
// If the frame is already 3-channel BGR, swapRB=true will turn it into RGB.
// If it is single-channel, convert to BGR first, then blobFromImage handles RB swap.
if (resized.channels() == 1)
    cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);

// blobFromImage:  BGR/GRAY → RGB  +  float32  +  CHW  (all in one go)
cv::Mat blob = cv::dnn::blobFromImage(
                  resized,                 // input image
                  1.f / 255.f,             // scale
                  cv::Size(netW, netH),
                  cv::Scalar(),            // mean
                  /*swapRB=*/true,         // BGR→RGB
                  /*crop=*/false,
                  CV_32F);                 // output type
// blob shape = [1×3×netH×netW]

// ─────────────────── 3. create ORT tensor (no copy) ─────────────────
std::array<int64_t, 4> inShape = {1, 3, netH, netW};
Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(
                              OrtArenaAllocator, OrtMemTypeDefault);

Ort::Value inTensor = Ort::Value::CreateTensor<float>(
                          memInfo,
                          blob.ptr<float>(),            // direct pointer
                          blob.total(),                 // element count
                          inShape.data(), inShape.size());


// ───────────────────── 3. inference ───────────────────────────────────────
const char* inNames[]  = { inputName_.c_str() };
const char* outNames[] = { outputName_.c_str() };

auto outs = session_->Run(Ort::RunOptions{nullptr},
                          inNames,  &inTensor, 1,
                          outNames, 1);

const float* p  = outs[0].GetTensorMutableData<float>();
const int64_t N = outs[0].GetTensorTypeAndShapeInfo().GetShape()[1]; // 300


std::vector<Detection> detections;
for (int i = 0; i < N; ++i, p += 6)
{
float x1 = p[0];
float y1 = p[1];
float x2 = p[2];
float y2 = p[3];
float conf = p[4];
int cls = static_cast<int>(p[5]);

if (conf < confThreshold_) continue;
if (cls < 0 || cls >= coco_class_names.size()) continue;

float x_scale = static_cast<float>(seqW) / netW;
float y_scale = static_cast<float>(seqH) / netH;

Detection d;
d.classId = cls;
d.confidence = conf;
d.label = coco_class_names[cls];
d.box = cv::Rect(
    cv::Point(int(std::round(x1 * x_scale)), int(std::round(y1 * y_scale))),
    cv::Point(int(std::round(x2 * x_scale)), int(std::round(y2 * y_scale)))
);


    detections.push_back(d);
}


return detections;


}


