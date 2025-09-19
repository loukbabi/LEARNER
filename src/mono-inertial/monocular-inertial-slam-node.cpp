#include "monocular-inertial-slam-node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <vector>


using std::placeholders::_1;
using ORB_SLAM3::IMU::Point;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* SLAM,
                                   const std::string& strDoEqual)
: Node("ORB_SLAM3_Mono_Inertial"), SLAM_(SLAM)
{
    std::stringstream ss_eq(strDoEqual);
    ss_eq >> std::boolalpha >> bClahe_;
    RCLCPP_INFO(this->get_logger(), "Equalisation (CLAHE): %s", bClahe_ ? "on" : "off");

    // Subscriptions
    subImu_ = this->create_subscription<ImuMsg>(
        "/imu", 1000, std::bind(&MonoInertialNode::GrabImu, this, _1));

    subImg_ = this->create_subscription<ImageMsg>(
        "camera", 100, std::bind(&MonoInertialNode::GrabImage, this, _1));

    // Sync thread
    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
}

MonoInertialNode::~MonoInertialNode()
{
    syncThread_->join();
    delete syncThread_;

    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

/* ---------- Callbacks ---------------------------------------------------- */

void MonoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bufMutex_);
    imuBuf_.push(msg);

    //RCLCPP_INFO(this->get_logger(), "IMU received: %.6f",
    // Utility::StampToSec(msg->header.stamp));
}

void MonoInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imgMutex_);
    if (!imgBuf_.empty()) imgBuf_.pop();
    imgBuf_.push(msg);
}

/* ---------- Helpers ------------------------------------------------------ */

cv::Mat MonoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr;

        // Accept any encoding and convert to MONO8 if needed
        if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
            return cv_ptr->image.clone();
        } else {
            cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
            cv::Mat gray;
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY); // BGR8/RGB8 -> GRAY
            return gray;
        }

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}


void MonoInertialNode::SyncWithImu()
{
    constexpr double maxTimeDiff = 0.01;
    constexpr double minImuWindow = 0.03; // Slightly faster sync

    while (rclcpp::ok()) {
        if (imgBuf_.empty() || imuBuf_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        double tImg;
        ImageMsg::SharedPtr imgMsg;

        {
            std::lock_guard<std::mutex> lock(imgMutex_);
            imgMsg = imgBuf_.front();
            tImg = Utility::StampToSec(imgMsg->header.stamp);
        }

        double tImuEarliest, tImuLatest;
        {
            std::lock_guard<std::mutex> lock(bufMutex_);
            tImuEarliest = Utility::StampToSec(imuBuf_.front()->header.stamp);
            tImuLatest   = Utility::StampToSec(imuBuf_.back()->header.stamp);
        }

        // Wait until IMU catches up
        if (tImg > tImuLatest) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // Skip image if no IMU available yet
        if (tImg < tImuEarliest) {
            std::lock_guard<std::mutex> lock(imgMutex_);
            imgBuf_.pop();
            continue;
        }

        // Get the image
        cv::Mat im;
        {
            std::lock_guard<std::mutex> lock(imgMutex_);
            im = GetImage(imgMsg);
            imgBuf_.pop();
        }

        if (im.empty()) continue;

        // Gather IMU up to tImg
        std::vector<Point> vImuMeas;
        {
            std::lock_guard<std::mutex> lock(bufMutex_);
            while (!imuBuf_.empty() &&
                   Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImg)
            {
                auto imu = imuBuf_.front();
                vImuMeas.emplace_back(
                    cv::Point3f(imu->linear_acceleration.x,
                                imu->linear_acceleration.y,
                                imu->linear_acceleration.z),
                    cv::Point3f(imu->angular_velocity.x,
                                imu->angular_velocity.y,
                                imu->angular_velocity.z),
                    Utility::StampToSec(imu->header.stamp));
                imuBuf_.pop();
            }
        }

        if (vImuMeas.empty()) continue;

        if (vImuMeas.front().t > tImg - minImuWindow) continue;

        if (bClahe_) clahe_->apply(im, im);

        try {
            Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tImg, vImuMeas);

            if (!Tcw.matrix().isZero(0)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Pose estimated at %.3f", tImg);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Exception in TrackMonocular: %s", e.what());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


