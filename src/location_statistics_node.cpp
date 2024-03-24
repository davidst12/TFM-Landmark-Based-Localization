#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "detection_msgs/msg/detection.hpp"
#include "detection_msgs/msg/detection_array.hpp"
#include "tfm_landmark_based_localization_package/msg/results.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"

using namespace std;

/*
 * Global variables
 */

geometry_msgs::msg::Pose real_car_pose;
detection_msgs::msg::DetectionArray detection_list;
tfm_landmark_based_localization_package::msg::Results localitation_result;

double linear_velocity = 0.0;
double angular_velocity = 0.0;

double current_error = 0.0;
double maximum_error = 0.0;
double average_error = 0.0;

int iteration = 0;
int recoveries_count = 0;

ofstream file;

/*
 * Global funtions
 */

// Error Statistics funtions
void calculate_error_in_cm()
{

	double l1 = pow(real_car_pose.position.x - localitation_result.car_position_estimation.position.x, 2);
	double l2 = pow(real_car_pose.position.y - localitation_result.car_position_estimation.position.y, 2);

	current_error = sqrt(l1 + l2) * 100;
}
void calculate_maximun_error()
{
	if (current_error > maximum_error)
	{
		maximum_error = current_error;
	}
}
void calculate_average_error()
{
	average_error = average_error * iteration / (iteration + 1) + current_error / (iteration + 1);
}

std::string format_double_with_comma(double input)
{
	std::string aux = to_string(input);
	for (unsigned int i = 0; i < aux.size(); i++)
	{
		if (aux[i] == '.')
		{
			aux[i] = ',';
		}
	}
	return aux;
}

// Print statistics in console
void print_results()
{
	cout << "--- Iteration " << iteration << " ---" << endl;

	if (localitation_result.recovery_applied)
	{
		recoveries_count++;
	}

	cout << "Real landmarks detection:      ";
	for (unsigned int i = 0; i < detection_list.detections.size(); i++)
	{
		cout << "[ " << detection_list.detections[i].id << " ] ";
	}
	cout << endl;

	cout << "Algorithm landmarks detection: ";
	for (unsigned int i = 0; i < localitation_result.detections_assignment.size(); i++)
	{
		cout << "[ " << localitation_result.detections_assignment[i] << " ] ";
	}
	cout << endl;

	cout << "Time spent in: LandmarkAssigment [ " << localitation_result.time_spent[0] << " micro s] ";
	cout << " , LocalitationAlgorithm [" << localitation_result.time_spent[1] << " micro s] " << endl;
	cout << "Assignment algorithm cost: " << localitation_result.cost << endl;

	cout << "Car real position:       ("
		 << real_car_pose.position.x << " , "
		 << real_car_pose.position.y << ")" << endl;
	cout << "Car estimated position:  ("
		 << localitation_result.car_position_estimation.position.x << " , "
		 << localitation_result.car_position_estimation.position.y << ")"
		 << " ; Angle: " << localitation_result.car_position_estimation.position.z
		 << endl;

	cout << "Current Error: " << current_error << " cm" << endl;
	cout << "Maximum Error: " << maximum_error << " cm" << endl;
	cout << "Average Error: " << average_error << " cm" << endl;
	cout << "Recovery Applied counter: " << recoveries_count << endl;
	cout << endl;
}

void print_results_in_file()
{
	std::string landmarks_detected = std::to_string(localitation_result.detections_assignment[0]);
	for (unsigned int i = 1; i < localitation_result.detections_assignment.size(); i++)
	{
		landmarks_detected += "-" + std::to_string(localitation_result.detections_assignment[i]);
	}

	std::string real_landmarks = detection_list.detections[0].id;
	for (unsigned int i = 1; i < detection_list.detections.size(); i++)
	{
		real_landmarks += "-" + detection_list.detections[i].id;
	}

	std::string location_estimated = "(" + to_string(localitation_result.car_position_estimation.position.x) + " | " + to_string(localitation_result.car_position_estimation.position.y) + ")";

	std::string real_location = "(" + to_string(real_car_pose.position.x) + " | " + to_string(real_car_pose.position.y) + ")";

	file
		<< iteration
		<< ";"
		<< localitation_result.detections_assignment.size()
		<< ";"
		<< landmarks_detected
		<< ";"
		<< real_landmarks
		<< ";"
		<< format_double_with_comma(localitation_result.time_spent[0])
		<< ";"
		<< format_double_with_comma(localitation_result.time_spent[1])
		<< ";"
		<< format_double_with_comma(localitation_result.cost)
		<< ";"
		<< localitation_result.recovery_applied
		<< ";"
		<< format_double_with_comma(linear_velocity)
		<< ";"
		<< format_double_with_comma(angular_velocity)
		<< ";"
		<< format_double_with_comma(localitation_result.car_position_estimation.position.x)
		<< ";"
		<< format_double_with_comma(localitation_result.car_position_estimation.position.y)
		<< ";"
		<< format_double_with_comma(localitation_result.car_position_estimation.position.z)
		<< ";"
		<< format_double_with_comma(real_car_pose.position.x)
		<< ";"
		<< format_double_with_comma(real_car_pose.position.y)
		<< ";"
		<< "0,0"
		<< ";"
		<< format_double_with_comma(current_error)
		<< ";"
		<< "0,0"
		<< endl;
}

void run_result_print_process()
{
	calculate_error_in_cm();
	calculate_maximun_error();
	calculate_average_error();
	print_results();
	print_results_in_file();
}

void print_header()
{
	file << "Iteration"
		 << ";"
		 << "Nº Landmarks detected"
		 << ";"
		 << "Landmarks detected"
		 << ";"
		 << "Real Landmarks"
		 << ";"
		 << "Association time (micro sec)"
		 << ";"
		 << "Localitation time (micro sec)"
		 << ";"
		 << "Association Cost"
		 << ";"
		 << "Recovery Applied"
		 << ";"
		 << "Linear Velocity (m/s)"
		 << ";"
		 << "Angular Velocity (rad/s)"
		 << ";"
		 << "Location estimated X (m)"
		 << ";"
		 << "Location estimated Y (m)"
		 << ";"
		 << "Location estimated Angle (rad)"
		 << ";"
		 << "Real location X (m)"
		 << ";"
		 << "Real location Y (m)"
		 << ";"
		 << "Real location Angle (rad)"
		 << ";"
		 << "Location Error (cm)"
		 << ";"
		 << "Orientation Error"
		 << endl;
}

// Three topics --> exact time policy callback
void sync_callback(
	const nav_msgs::msg::Odometry::SharedPtr odometry_message,
	const detection_msgs::msg::DetectionArray::SharedPtr detections_array,
	const tfm_landmark_based_localization_package::msg::Results::SharedPtr result)
{
	detection_list = *detections_array;

	localitation_result = *result;

	real_car_pose = odometry_message->pose.pose;
	linear_velocity = odometry_message->twist.twist.linear.x;
	angular_velocity = odometry_message->twist.twist.angular.z;

	run_result_print_process();

	iteration++;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Node + NodeObjects declaration
	auto location_statistics_node = std::make_shared<rclcpp::Node>("location_statistics_node");

	message_filters::Subscriber<nav_msgs::msg::Odometry> string_sub(location_statistics_node, "/carla/ego_vehicle/odometry");
	message_filters::Subscriber<detection_msgs::msg::DetectionArray> bool_sub(location_statistics_node, "/fake_pole_detection");
	message_filters::Subscriber<tfm_landmark_based_localization_package::msg::Results> bool_sub2(location_statistics_node, "/results");

	// exact time policy
	typedef message_filters::sync_policies::ExactTime<
		nav_msgs::msg::Odometry,
		detection_msgs::msg::DetectionArray,
		tfm_landmark_based_localization_package::msg::Results>
		exact_policy;
	message_filters::Synchronizer<exact_policy> syncExact(exact_policy(25), string_sub, bool_sub, bool_sub2);

	// register the exact time callback
	syncExact.registerCallback(sync_callback);

	// Open statistics file
	std::string file_name = "file_name.csv";
	std::string package_share_directory = ament_index_cpp::get_package_share_directory("tfm_landmark_based_localization_package");
	std::string statistics_directory = package_share_directory + "/../../../../src/tfm_landmark_based_localization_package/statistics/" + file_name;
	file.open(statistics_directory);
	print_header();

	rclcpp::spin(location_statistics_node);

	file.close();

	rclcpp::shutdown();
	return 0;
}