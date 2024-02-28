#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "detection_msgs/msg/detection.hpp"
#include "detection_msgs/msg/detection_array.hpp"
#include "tfm_landmark_based_localization_package/msg/results.hpp"
#include "detection_msgs/msg/car_pose_for_fake_detections.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include <chrono>
#include <thread>   
#include <vector>

using namespace std;

geometry_msgs::msg::Pose realCarPose;
detection_msgs::msg::DetectionArray detectionList;
tfm_landmark_based_localization_package::msg::Results singleResult;
bool isNewDetectionReceived = false;
int recoveriesCount = 0;
double currentError = 0.0;
double maximumError = 0.0;
double averageError = 0.0;

int iteration = 0;
int localitationResultCallbackSeq = -1;
int carPoseForFakeiteration = -1;


void calculateErrorInCm(){

  double l1 = pow(realCarPose.position.x - singleResult.car_position_estimation.position.x, 2);
  double l2 = pow(realCarPose.position.y - singleResult.car_position_estimation.position.y, 2);

  currentError = sqrt(l1+l2)*100;
}
void calculateMaximunError(){
  if(currentError > maximumError){
    maximumError = currentError;
  }
}
void calculateAverageError(){
  averageError = averageError*iteration/(iteration+1)+currentError/(iteration+1);
}

void printResults(){

  if(singleResult.recovery_applied){
    recoveriesCount++;
  }

  cout << "--- Iteration " << iteration << " ---" << endl;
  cout << "Real landmarks detection:      ";
  for (unsigned int i = 0; i<detectionList.detections.size(); i++){
    cout << "[ " << detectionList.detections[i].id << " ] ";
  }
  cout << endl;
  cout << "Algorithm landmarks detection: ";
  for (unsigned int i = 0; i<singleResult.detections_assignment.size(); i++){
    cout << "[ " << singleResult.detections_assignment[i] << " ] ";
  }
  cout << endl;
  cout << "Time spent in: LandmarkAssigment [ " << singleResult.time_spent[0] << " micro s] ";
  cout << " , LocalitationAlgorithm [" << singleResult.time_spent[1] << " micro s] " << endl;
  cout << "Assignment algorithm cost: " << singleResult.cost << endl;
  cout << "Car real position:       (" << realCarPose.position.x << " , " << realCarPose.position.y << ")" << endl;
  cout << "Car estimated position:  (" << singleResult.car_position_estimation.position.x << " , " << singleResult.car_position_estimation.position.y << ")" << endl;
  cout << "Current Error: " << currentError << " cm" << endl;
  cout << "Maximum Error: " << maximumError << " cm" << endl;
  cout << "Average Error: " << averageError << " cm" << endl;
  cout << "Recovery Applied counter: " << recoveriesCount << endl;
  cout << endl;

}

void runResultPrintProcess(){
  calculateErrorInCm();
  calculateMaximunError();
  calculateAverageError();
  printResults();
}

void sync_callback(const nav_msgs::msg::Odometry::SharedPtr odometryMessage, const detection_msgs::msg::DetectionArray::SharedPtr detectionsArray, const tfm_landmark_based_localization_package::msg::Results::SharedPtr localitationResult)
{
  //std::cout<<"Received msg: "<<std::endl;
  detectionList = *detectionsArray;

  isNewDetectionReceived = true;
  singleResult = *localitationResult;

  realCarPose = odometryMessage->pose.pose;

  runResultPrintProcess();

  iteration++;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Node + NodeObjects declaration
  auto resultsNode = std::make_shared<rclcpp::Node>("results_node");

  // create subscribers to the topics of interest
  message_filters::Subscriber<nav_msgs::msg::Odometry> string_sub(resultsNode, "/carla/ego_vehicle/odometry");
  message_filters::Subscriber<detection_msgs::msg::DetectionArray> bool_sub(resultsNode, "/fake_pole_detection");
  message_filters::Subscriber<tfm_landmark_based_localization_package::msg::Results> bool_sub2(resultsNode, "/results");

  // exact time policy
  typedef message_filters::sync_policies::ExactTime<nav_msgs::msg::Odometry, detection_msgs::msg::DetectionArray, tfm_landmark_based_localization_package::msg::Results> exact_policy;
  message_filters::Synchronizer<exact_policy>syncExact(exact_policy(20), string_sub, bool_sub, bool_sub2);

  // register the exact time callback
  syncExact.registerCallback(sync_callback);


  rclcpp::spin(resultsNode);
  
  rclcpp::shutdown();
  return 0;
}