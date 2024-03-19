#include <g2o/types/slam2d/se2.h>

#include <chrono>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "detection_msgs/msg/detection.hpp"
#include "detection_msgs/msg/detection_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include "utils/errors.cpp"
#include "utils/utils.cpp"

using namespace std;

/*
 * Global variables
 */

rclcpp::Publisher<detection_msgs::msg::DetectionArray>::SharedPtr detections_publisher;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;

g2o::SE2 vehicle_pose = g2o::SE2(0.0, 0.0, 0.0);
std::vector<utils::LandmarkObject> landmark_poses;

int global_radius_detection;
float global_min_noise_error_sigma;
float global_max_noise_error_sigma;

auto detections_array = detection_msgs::msg::DetectionArray();

/*
 * Global functions
 */

// Carla Odometry Callback
void message_callback(const nav_msgs::msg::Odometry::SharedPtr odometryMessage)
{
	double poseX = odometryMessage->pose.pose.position.x;
	double poseY = odometryMessage->pose.pose.position.y;

	vehicle_pose = g2o::SE2(poseX, poseY, 0.0);

	detections_array = detection_msgs::msg::DetectionArray();
	detections_array.header = odometryMessage->header;
}

// Calculate landmarks measurement with noise
bool compute_landmark_measurement(const g2o::SE2 &pose,
								  utils::LandmarkObject landmark,
								  Eigen::Vector2d &measurement,
								  Eigen::Matrix2d &inf_matrix)
{
	// Compute the perfect measurement
	Eigen::Vector2d true_measurement = pose.inverse() * landmark.get_pose();

	// Get module vector
	double vector_module = true_measurement.norm();

	if (vector_module < global_radius_detection)
	{
		// Calculate dinamic sigma error
		double dynamic_sigma_error =
			((global_max_noise_error_sigma - global_min_noise_error_sigma) /
			 global_radius_detection) *
				vector_module +
			global_min_noise_error_sigma;

		// Get gaussian error
		double error = errors::gaussian(dynamic_sigma_error);

		// Add gaussian noise
		measurement = true_measurement + Eigen::Vector2d{error, error};

		// Fill the information matrix
		inf_matrix = Eigen::Matrix2d::Identity() * pow(error, 2);

		return true;
	}
	return false;
}

// Get fake detected landmarks
detection_msgs::msg::DetectionArray get_detections()
{
	cout << "Current car pose: ( " << vehicle_pose[0] << " , " << vehicle_pose[1] << " ) " << endl;

	for (unsigned int i = 0; i < landmark_poses.size(); i++)
	{
		Eigen::Vector2d measurement;
		Eigen::Matrix2d inf_matrix;

		bool landmarkIsInRange = compute_landmark_measurement(
			vehicle_pose, landmark_poses[i], measurement, inf_matrix);

		if (landmarkIsInRange)
		{
			auto detection = detection_msgs::msg::Detection();
			detection.id = landmark_poses[i].get_id();
			detection.position.x = measurement[0];
			detection.position.y = measurement[1];
			cout << "  Landmark detection [" << i << "] : " << measurement[0] << " , " << measurement[1] << endl;

			detection.covariance = utils::matrix2d_to_covariance_arrayX9(inf_matrix);

			detections_array.detections.push_back(detection);
		}
	}

	cout << "Number of detections: " << detections_array.detections.size() << endl;

	return detections_array;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Node + NodeObjects declaration
	auto fake_pole_detection_node = std::make_shared<rclcpp::Node>("fake_pole_detection_node");

	detections_publisher = fake_pole_detection_node->create_publisher<detection_msgs::msg::DetectionArray>(
		"fake_pole_detection", 1);
	odometry_subscription = fake_pole_detection_node->create_subscription<nav_msgs::msg::Odometry>(
		"/carla/ego_vehicle/odometry", 1, message_callback);

	// Globar parameters -> 1º declare them, 2º get them
	fake_pole_detection_node->declare_parameter("radius_detection", 0);
	fake_pole_detection_node->declare_parameter("min_noise_error_sigma", 0.0);
	fake_pole_detection_node->declare_parameter("max_noise_error_sigma", 0.0);
	fake_pole_detection_node->declare_parameter("package_prefix", "");
	fake_pole_detection_node->declare_parameter("landmarks_ground_truth_file", "");

	fake_pole_detection_node->get_parameter("radius_detection", global_radius_detection);
	fake_pole_detection_node->get_parameter("min_noise_error_sigma", global_min_noise_error_sigma);
	fake_pole_detection_node->get_parameter("max_noise_error_sigma", global_max_noise_error_sigma);
	auto package_prefix = fake_pole_detection_node->get_parameter("package_prefix").as_string();
	auto landmarks_ground_truth_file = fake_pole_detection_node->get_parameter("landmarks_ground_truth_file").as_string();
	cout << "Global parameters" << endl;
	cout << " - Radius_detection -> " << global_radius_detection << endl;
	cout << " - Noise_error_sigma_min -> " << global_min_noise_error_sigma << endl;
	cout << " - Noise_error_sigma_max -> " << global_max_noise_error_sigma << endl
		 << endl;

	// Get GroundTruth Landmarks
	landmark_poses = utils::get_landmark_poses(package_prefix, landmarks_ground_truth_file);

	// Run every 0.1 second
	rclcpp::WallRate rate(10);

	while (rclcpp::ok())
	{
		cout << "-----------------" << endl;

		// Get car pose
		rclcpp::spin_some(fake_pole_detection_node);

		// Get detections
		auto detectionsArray = get_detections();

		// Publish detections
		detections_publisher->publish(detectionsArray);

		rate.sleep();
	}

	detections_publisher.reset();

	rclcpp::shutdown();
	return 0;
}