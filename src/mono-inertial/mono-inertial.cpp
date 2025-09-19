#include "monocular-inertial-slam-node.hpp"
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "System.h"      // ORB-SLAM3 core

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "\nUsage:\n"
                  << "  ros2 run orbslam3 mono_inertial "
                  << "path_to_vocabulary  path_to_settings  [do_equalize]\n";
        rclcpp::shutdown();
        return 1;
    }
    if (argc == 3) argv[3] = const_cast<char *>("false");   // default equalise = false

    rclcpp::init(argc, argv);

    const bool use_viewer = true;
    ORB_SLAM3::System SLAM(
        argv[1],                 // vocabulary
        argv[2],                 // settings
        ORB_SLAM3::System::IMU_MONOCULAR,   
        use_viewer);

    auto node = std::make_shared<MonoInertialNode>(&SLAM, argv[3]);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

