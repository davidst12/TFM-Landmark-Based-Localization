#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "detection_msgs/msg/detection.hpp"
#include "detection_msgs/msg/detection_array.hpp"
#include <chrono>
#include <thread>   
#include <vector>
#include "utils/utils.cpp"
#include "utils/errors.cpp"

#include <g2o/types/slam2d/se2.h>

using namespace std;

// Ros Publisher object (topic: /fake_pole_detection)
rclcpp::Publisher<detection_msgs::msg::DetectionArray>::SharedPtr publisher;

g2o::SE2 vehiclePose = g2o::SE2(0.0, 0.0, 0.0);
std::vector<utils::LandmarkObject> landmarkPoses;

int global_radius_detection;
float global_noise_error_sigma;

//** Global functions **//

// Car Odometry message
void messageCallback(const nav_msgs::msg::Odometry::SharedPtr odometryMessage) {
  double poseX = odometryMessage->pose.pose.position.x;
  double poseY = odometryMessage->pose.pose.position.y;
  //cout << "Current car pose: ( " << poseX << "," << poseY << " )" << endl;
  vehiclePose = g2o::SE2(poseX, poseY, 0.0);
}

// Calculate measurement with noise
bool compute_landmark_measurement(const g2o::SE2& pose, utils::LandmarkObject landmark,
                                  Eigen::Vector2d& measurement, Eigen::Matrix2d& inf_matrix)
{
    // Compute the perfect measurement
    Eigen::Vector2d true_measurement = pose.inverse() * landmark.getPose();

    double vectorModule = true_measurement.norm();
    double radiousDetection = global_radius_detection;

    if(vectorModule < radiousDetection){
        // Add gaussian noise
        measurement = true_measurement + Eigen::Vector2d{
          errors::gaussian(global_noise_error_sigma), 
          errors::gaussian(global_noise_error_sigma)
        };
        // Fill the information matrix
        inf_matrix = Eigen::Matrix2d::Identity() * 10;

        return true;
    }
    return false;
}

// Calculate fake detected landmarks
detection_msgs::msg::DetectionArray getDetections(){
  cout << "Current car pose: ( " << vehiclePose[0] << " , " << vehiclePose[1] << " ) " << endl;
  auto detectionsArray = detection_msgs::msg::DetectionArray();

  for(unsigned int i = 0; i<landmarkPoses.size(); i++){

    Eigen::Vector2d measurement;
    Eigen::Matrix2d inf_matrix;
    bool landmarkIsInRange = compute_landmark_measurement(vehiclePose, landmarkPoses[i], measurement, inf_matrix);

    if (landmarkIsInRange){

      auto detection = detection_msgs::msg::Detection();
      detection.id = landmarkPoses[i].getId();
      detection.position.x = measurement[0];
      detection.position.y = measurement[1];
      cout << "  Landmark detection [" << i << "] : " << measurement[0] << " , " <<measurement[1] << endl;

      detectionsArray.detections.push_back(detection);
    }
  }

  cout << "Number of detections: " << detectionsArray.detections.size() << endl;

  return detectionsArray;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Node + NodeObjects declaration
  auto fakePoleDetectionNode = std::make_shared<rclcpp::Node>("fake_pole_detection_node");
  publisher = fakePoleDetectionNode->create_publisher<detection_msgs::msg::DetectionArray>("fake_pole_detection", 1);
  auto subscription = fakePoleDetectionNode->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 1, messageCallback);
  
  // Globar parameters -> 1º declare, 2º get them
  fakePoleDetectionNode->declare_parameter("radius_detection", 0);
  fakePoleDetectionNode->declare_parameter("noise_error_sigma", 0.0);
  fakePoleDetectionNode->get_parameter("radius_detection", global_radius_detection);
  fakePoleDetectionNode->get_parameter("noise_error_sigma", global_noise_error_sigma);
  cout << "Global parameters" << endl;
  cout << " - Radius_detection -> " << global_radius_detection << endl;
  cout << " - Noise_error_sigma -> " << global_noise_error_sigma << endl << endl;

  // Get GroundTruth Landmarks
  landmarkPoses = utils::getLandmarkPoses();

  // Run every 1 second
  rclcpp::WallRate rate(1);  
  while (rclcpp::ok()) {

    cout << "-----------------" <<endl;

    // Get car pose
    rclcpp::spin_some(fakePoleDetectionNode);

    // Get detections
    auto detectionsArray = getDetections();

    // Publish detections
    publisher->publish(detectionsArray);

    rate.sleep();
  }  

  // Si no salta error
  publisher.reset();
  
  rclcpp::shutdown();
  return 0;
}