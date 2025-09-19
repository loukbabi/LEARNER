#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include <Eigen/Geometry>

// These headers are needed for the file-writing logic
#include "Tracking.h" // Needed for Tracking::OK

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

// Global variable to keep track of the last time we saved the heavy path files
auto last_map_save = std::chrono::steady_clock::now();

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (trajectory_file_name)" << endl;
        return 1;
    }

    // --- This section is for loading image data and remains unchanged ---
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestampsCam);
    int nImages = vstrImageFilenames.size();
    // --- End of data loading ---

    // Create SLAM system.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cout << endl << "-------" << endl;
    cout << "Start processing sequence..." << endl;

    // Main loop
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        cv::Mat im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestampsCam[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        auto t1 = std::chrono::steady_clock::now();

// --- BEGINNING OF DEFINITIVE BLOCK TO ADD/REPLACE ---

// Pass the image to the SLAM system AND catch the returned pose in one step.
Sophus::SE3f Tcw_sophus = SLAM.TrackMonocular(im, tframe);

// Only save data if tracking is successful and the pose is valid
if(SLAM.GetTrackingState() == ORB_SLAM3::Tracking::OK && !Tcw_sophus.matrix().isZero())
{
    // --- 1. Save Current Pose (ALWAYS) ---
    {
        Eigen::Matrix4f Tcw_eigen = Tcw_sophus.matrix();
        cv::Mat Tcw(4, 4, CV_32F);
        for(int i=0; i<4; i++) for(int j=0; j<4; j++) Tcw.at<float>(i,j) = Tcw_eigen(i,j);
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0,3).col(3);

        Eigen::Matrix3d rot_matrix_eigen;
        rot_matrix_eigen << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
                             Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
                             Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);

        Eigen::Quaterniond q(rot_matrix_eigen);

        std::ofstream pose_file("/tmp/rover_slam_pose.txt");
        if (pose_file.is_open())
        {
            pose_file << std::fixed << std::setprecision(6)
                      << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " "
                      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
    }

    // --- 2. Save Map and Path (EVERY 4 SECONDS) ---
    auto now = std::chrono::steady_clock::now();
    double time_since_last_save = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_map_save).count();

    if (time_since_last_save > 4.0)
    {
        // Use the library's OWN save function for the trajectory. This is efficient.
        SLAM.SaveKeyFrameTrajectoryTUM("/tmp/rover_slam_path.txt");
        
        // Use our new public "front door" functions to get the map data
        std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetAllMapPoints();
        std::ofstream map_file("/tmp/rover_slam_map_points.txt");
        if (map_file.is_open())
        {
            for(ORB_SLAM3::MapPoint* pMP : map_points) {
                if(pMP && !pMP->isBad()) {
                    Eigen::Vector3f pos = pMP->GetWorldPos();
                    map_file << std::fixed << std::setprecision(4)
                             << pos(0) << " " << pos(1) << " " << pos(2) << std::endl;
                }
            }
        }
        
        last_map_save = now; // Reset the timer
    }
}
// --- END OF DEFINITIVE BLOCK ---


        auto t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // Wait to load the next frame
        if(ni < nImages-1)
        {
            double T = vTimestampsCam[ni+1] - tframe;
            if(ttrack < T)
                usleep((T-ttrack)*1e6);
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save final trajectory to a permanent file
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);
        }
    }
}

