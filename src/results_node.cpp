#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <thread>
#include <vector>

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

double current_error = 0.0;
double maximum_error = 0.0;
double average_error = 0.0;

int iteration = 0;
int recoveries_count = 0;

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
		 << endl;

	cout << "Current Error: " << current_error << " cm" << endl;
	cout << "Maximum Error: " << maximum_error << " cm" << endl;
	cout << "Average Error: " << average_error << " cm" << endl;
	cout << "Recovery Applied counter: " << recoveries_count << endl;
	cout << endl;
}

void run_result_print_process()
{
	calculate_error_in_cm();
	calculate_maximun_error();
	calculate_average_error();
	print_results();
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

	run_result_print_process();

	iteration++;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Node + NodeObjects declaration
	auto results_node = std::make_shared<rclcpp::Node>("results_node");

	message_filters::Subscriber<nav_msgs::msg::Odometry> string_sub(results_node, "/carla/ego_vehicle/odometry");
	message_filters::Subscriber<detection_msgs::msg::DetectionArray> bool_sub(results_node, "/fake_pole_detection");
	message_filters::Subscriber<tfm_landmark_based_localization_package::msg::Results> bool_sub2(results_node, "/results");

	// exact time policy
	typedef message_filters::sync_policies::ExactTime<
		nav_msgs::msg::Odometry,
		detection_msgs::msg::DetectionArray,
		tfm_landmark_based_localization_package::msg::Results>
		exact_policy;
	message_filters::Synchronizer<exact_policy> syncExact(exact_policy(25), string_sub, bool_sub, bool_sub2);

	// register the exact time callback
	syncExact.registerCallback(sync_callback);

	rclcpp::spin(results_node);

	rclcpp::shutdown();
	return 0;
}