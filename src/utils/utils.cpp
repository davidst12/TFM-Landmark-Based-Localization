#include <string>
#include <fstream>
#include <vector> 
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace utils{

    using json = nlohmann::json;

    // Object to save groundtruth landmarks
    class LandmarkObject {
        public:
        LandmarkObject(std::string id, double poseX, double poseY){
            _id = id;
            _pose[0] = poseX;
            _pose[1] = poseY;
        }
        std::string getId(){ return _id; }
        Eigen::Vector2d getPose(){ return _pose; }

        private:
        std::string _id;
        Eigen::Vector2d _pose;
    };

    // Get landmarks from config file
    static std::vector<LandmarkObject> getLandmarkPoses(){

        // Path result: /hdd/ros2_tfm_ws/install/tfm_landmark_based_localization_package
        std::string package_path = ament_index_cpp::get_package_prefix("tfm_landmark_based_localization_package");
        std::ifstream f(package_path+"/../../src/tfm_landmark_based_localization_package/config/landmark_poses_town02.json");

        json data = json::parse(f);
        std::vector<std::vector<int>> l = data["landmarks"];
        std::vector<LandmarkObject> landmarks_array;

        for(unsigned int i = 0; i<l.size(); i++){
            LandmarkObject landmark = LandmarkObject(std::to_string(i), l[i][0], l[i][1]);
            landmarks_array.push_back(landmark);
        }
        std::cout << "Number of landmarks: "<<landmarks_array.size()<< std::endl;

        return landmarks_array;
    }

}

