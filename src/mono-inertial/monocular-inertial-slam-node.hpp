#ifndef __MONO_INERTIAL_NODE_HPP__
#define __MONO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "utility.hpp"

using ImuMsg   = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonoInertialNode : public rclcpp::Node
{
public:
    MonoInertialNode(ORB_SLAM3::System* pSLAM,
                     const std::string& strDoEqual);
    ~MonoInertialNode();

private:
    // ROS callbacks
    void GrabImu   (const ImuMsg::SharedPtr msg);
    void GrabImage (const ImageMsg::SharedPtr msg);

    // Helpers
    cv::Mat GetImage (const ImageMsg::SharedPtr msg);
    void    SyncWithImu();

    // Subscribers
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;

    // SLAM back-end
    ORB_SLAM3::System* SLAM_;

    // Buffers & thread
    std::queue<ImuMsg::SharedPtr>   imuBuf_;
    std::queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutex_, imgMutex_;
    std::thread* syncThread_;

    // Options
    bool           bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8,8));
};

#endif  // __MONO_INERTIAL_NODE_HPP__

