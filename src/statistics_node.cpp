#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "utils/utils.cpp"

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
double real_orientation = 0.0;

double current_error = 0.0;
double maximum_error = 0.0;
double average_error = 0.0;
double orientation_error = 5.0;
double average_detection_error = 0.0;

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
void calculate_orientation_error_in_rad()
{
	orientation_error = abs(abs(real_orientation) - abs(localitation_result.car_position_estimation.position.z));
}
void average_measurement_error()
{
	double detection_error_sum = 0.0;
	for (unsigned int i = 0; i < detection_list.detections.size(); i++)
	{
		detection_error_sum += sqrt(pow(detection_list.detections[i].covariance[0], 2) + pow(detection_list.detections[i].covariance[4], 2));
	}
	average_detection_error = detection_error_sum / detection_list.detections.size();
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
		 << real_car_pose.position.y << ")"
		 << " ; Angle: " << real_orientation
		 << endl;
	cout << "Car estimated position:  ("
		 << localitation_result.car_position_estimation.position.x << " , "
		 << localitation_result.car_position_estimation.position.y << ")"
		 << " ; Angle: " << localitation_result.car_position_estimation.position.z
		 << endl;

	cout << "Current Error: " << current_error << " cm" << endl;
	cout << "Maximum Error: " << maximum_error << " cm" << endl;
	cout << "Average Error: " << average_error << " cm" << endl;
	cout << "Orientation Error: " << orientation_error << " rad" << endl;
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

	string landmarks_detection_match = (landmarks_detected == real_landmarks || landmarks_detected == "-1") ? "YES" : "NO";

	file
		<< iteration
		<< ";"
		<< detection_list.detections.size()
		<< ";"
		<< landmarks_detected
		<< ";"
		<< real_landmarks
		<< ";"
		<< landmarks_detection_match
		<< ";"
		<< format_double_with_comma(average_detection_error)
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
		<< format_double_with_comma(real_orientation)
		<< ";"
		<< format_double_with_comma(current_error)
		<< ";"
		<< format_double_with_comma(orientation_error)
		<< endl;
}

void run_result_print_process()
{
	calculate_error_in_cm();
	calculate_maximun_error();
	calculate_average_error();
	calculate_orientation_error_in_rad();
	average_measurement_error();
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
		 << "Landmarks detections match"
		 << ";"
		 << "Average measurement error"
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
		 << "Orientation Error (rad)"
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
	real_orientation = utils::yaw_from_quaternion<double>(real_car_pose.orientation);

	run_result_print_process();

	iteration++;
}

std::string generate_file_name()
{
	time_t now = time(0);
	struct tm tstruct;
	char buf[80];
	tstruct = *localtime(&now);

	strftime(buf, sizeof(buf), "test-%Y-%m-%d--", &tstruct);

	std::string f = buf + std::to_string(tstruct.tm_hour) + "-" + std::to_string(tstruct.tm_min) + "-" + std::to_string(tstruct.tm_sec) + ".csv";

	return f;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Node + NodeObjects declaration
	auto statistics_node = std::make_shared<rclcpp::Node>("statistics_node");

	message_filters::Subscriber<nav_msgs::msg::Odometry> string_sub(statistics_node, "/carla/ego_vehicle/odometry");
	message_filters::Subscriber<detection_msgs::msg::DetectionArray> bool_sub(statistics_node, "/fake_pole_detection");
	message_filters::Subscriber<tfm_landmark_based_localization_package::msg::Results> bool_sub2(statistics_node, "/results");

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
	std::string file_name = generate_file_name();
	std::string package_share_directory = ament_index_cpp::get_package_share_directory("tfm_landmark_based_localization_package");
	std::string statistics_directory = package_share_directory + "/../../../../src/tfm_landmark_based_localization_package/statistics/" + file_name;
	file.open(statistics_directory);
	print_header();

	rclcpp::spin(statistics_node);

	file.close();

	rclcpp::shutdown();
	return 0;
}