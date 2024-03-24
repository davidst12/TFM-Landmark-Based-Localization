#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "utils/utils.cpp"
#include "utils/errors.cpp"
#include "detection_msgs/msg/detection_array.hpp"
#include "detection_msgs/msg/detection.hpp"
#include "tfm_landmark_based_localization_package/msg/results.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <nlohmann/json.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include <cmath>

#include "association_algorithm/Hungarian.h"

using json = nlohmann::json;
using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LandmarksGraphLocationNode : public rclcpp::Node
{
public:
    // Constructor
    LandmarksGraphLocationNode() : Node("landmarks_graph_location_node", rclcpp::NodeOptions())
    {
        subscription_ = this->create_subscription<detection_msgs::msg::DetectionArray>(
            "/fake_pole_detection", 1, std::bind(&LandmarksGraphLocationNode::detectionsCallback, this, _1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/carla/ego_vehicle/gnss", 1, std::bind(&LandmarksGraphLocationNode::gnss_callback, this, _1));
        publisher = this->create_publisher<tfm_landmark_based_localization_package::msg::Results>("/results", 1);

        // Global parameters
        declare_parameter("use_landmark_assignment_algorithm", false);
        declare_parameter("gps_noise_error_sigma", 0.0);
        declare_parameter("package_prefix", "");
        declare_parameter("landmarks_ground_truth_file", "");
        global_use_landmark_assignment_algorithm = get_parameter("use_landmark_assignment_algorithm").as_bool();
        global_gps_noise_error_sigma = get_parameter("gps_noise_error_sigma").as_double();
        auto package_prefix = get_parameter("package_prefix").as_string();
        auto landmarks_ground_truth_file = get_parameter("landmarks_ground_truth_file").as_string();

        landmark_poses = utils::get_landmark_poses(package_prefix, landmarks_ground_truth_file);

        cout << "Global parameters" << endl;
        cout << " - Use_landmark_assignment_algorithm -> " << global_use_landmark_assignment_algorithm << endl;
        cout << " - Gps_noise_error_sigma -> " << global_gps_noise_error_sigma << endl;

        recovery_counter = 0;
    }

    // Wait for initial pose from user
    void wait_for_initial_pose()
    {
        cout << "Please, enter initial pose X and Y of the car" << endl;
        string pose_x;
        string pose_y;
        cin >> pose_x;
        cin >> pose_y;
        cout << "Initial car pose: ( " << pose_x << " , " << pose_y << " )" << endl;
        vehicle_pose = g2o::SE2(stod(pose_x), stod(pose_y), 0.0);
    }

private:
    rclcpp::Subscription<detection_msgs::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription2_;
    rclcpp::Publisher<tfm_landmark_based_localization_package::msg::Results>::SharedPtr publisher;

    std::vector<utils::LandmarkObject> landmark_poses;
    std::vector<detection_msgs::msg::Detection> detections;

    g2o::SE2 vehicle_pose;
    g2o::SE2 gnss_vehicle_pose;
    bool global_use_landmark_assignment_algorithm;
    double global_gps_noise_error_sigma;

    tfm_landmark_based_localization_package::msg::Results results;
    int recovery_counter;

    void cleanResults()
    {
        results.car_position_estimation = geometry_msgs::msg::Pose();
        results.detections_assignment = vector<int>();
        results.time_spent = vector<double>();
        results.cost = 0.0;
    }

    // Callback of "/fake_pole_detection" topic
    void detectionsCallback(const detection_msgs::msg::DetectionArray &detections_list)
    {
        cout << " - Detections received: " << detections_list.detections.size() << endl;

        detections.clear();
        cleanResults();

        for (unsigned int i = 0; i < detections_list.detections.size(); i++)
        {
            detections.push_back(detections_list.detections[i]);
        }

        results.header = detections_list.header;

        estimate_localitation();
    }

    // Callback of "/carla/ego_vehicle/gnss" topic
    void gnss_callback(const sensor_msgs::msg::NavSatFix &gnssValue)
    {

        if (recovery_counter > 1)
        {
            double lat = gnssValue.latitude;
            double lon = gnssValue.longitude;

            double y = 6371000 * tan(lat * 3.1415 / 180) + errors::gaussian(global_gps_noise_error_sigma);
            double x = 6371000 * tan(lon * 3.1415 / 180) + errors::gaussian(global_gps_noise_error_sigma);

            gnss_vehicle_pose = g2o::SE2(x, y, 0.0);
        }
    }

    // Distance: car -> landmark_detected
    double calculate_distance(Eigen::Vector2d landmarkPose, g2o::SE2 poseCar, detection_msgs::msg::Detection measure)
    {

        double l1 = pow(landmarkPose[0] - (poseCar[0] + measure.position.x), 2);
        double l2 = pow(landmarkPose[1] - (poseCar[1] + measure.position.y), 2);

        return sqrt(l1 + l2);
    }

    // Association Algorithm
    vector<int> assignment_landmark_algorithm()
    {
        cout << "-----------------" << endl
             << endl;
        cout << "Landmark - Measure association algorithm (Hungarian Method)" << endl;

        // Start timer
        utils::tic();

        // Fill Cost Matrix
        vector<vector<double>> costMatrix(landmark_poses.size(), vector<double>(detections.size(), 0.0));

        for (unsigned int i = 0; i < landmark_poses.size(); i++)
        {
            for (unsigned int j = 0; j < detections.size(); j++)
            {
                double a = calculate_distance(landmark_poses[i].get_pose(), vehicle_pose, detections[j]);
                costMatrix[i][j] = a;
            }
        }

        // Run algorithm
        HungarianAlgorithm HungAlgo;
        vector<int> assignment;
        vector<int> finalAssignment;

        double cost = HungAlgo.Solve(costMatrix, assignment);

        // Associate landmark - measure
        for (unsigned int landmarkEntry = 0; landmarkEntry < costMatrix.size(); landmarkEntry++)
        {
            int measureEntry = assignment[landmarkEntry];
            if (measureEntry != -1)
            {
                finalAssignment.push_back(landmarkEntry);
                std::cout << " - Landmark [" << landmarkEntry << "] -> Measure " << measureEntry << endl;
            }
        }
        std::cout << " - Cost: " << cost << std::endl;

        if (cost > 50)
        {
            recovery_counter++;
        }
        else
        {
            recovery_counter = 0;
        }

        // End timer
        double timeSpent = utils::tic();
        cout << "-----> Time spent in Association Algorithm: " << timeSpent << " micros" << endl;

        results.detections_assignment = finalAssignment;
        results.cost = cost;
        results.time_spent.push_back(timeSpent);

        return finalAssignment;
    }

    // Location based in graph algorithm
    void estimate_localitation()
    {

        // If recovery is true -> vehicle_pose lo cojo del GNSS
        results.recovery_applied = false;
        if (recovery_counter == 3)
        {
            recovery_counter = 0;
            vehicle_pose = gnss_vehicle_pose;
            results.recovery_applied = true;
        }

        vector<int> landmarkAssignment;
        if (global_use_landmark_assignment_algorithm)
        {
            landmarkAssignment = assignment_landmark_algorithm();
        }

        cout << endl
             << "Graph Localitation Algorithm" << endl;

        // Start timer
        utils::tic();

        // Setup optimizer algorithm and solver
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(true);

        using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
        using SlamLinearSolver = g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType>;
        auto linearSolver = std::make_unique<SlamLinearSolver>();
        linearSolver->setBlockOrdering(false);
        g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(std::make_unique<SlamBlockSolver>(std::move(linearSolver)));
        optimizer.setAlgorithm(solver);

        // Set fixed vertices from map of landmarks
        for (size_t i = 0; i < landmark_poses.size(); i++)
        {
            g2o::VertexPointXY *landmark = new g2o::VertexPointXY;
            landmark->setId(i);
            landmark->setFixed(true);
            landmark->setEstimate(landmark_poses[i].get_pose());
            optimizer.addVertex(landmark);
        }

        // Set vertex from vehicle pose (initial guess)
        g2o::VertexSE2 *vehicle_pose_vertex = new g2o::VertexSE2;
        vehicle_pose_vertex->setId(1000);
        vehicle_pose_vertex->setEstimate(vehicle_pose);
        optimizer.addVertex(vehicle_pose_vertex);

        // Set detections (graph edges)
        for (size_t i = 0; i < detections.size(); i++)
        {

            g2o::EdgeSE2PointXY *landmark_observation = new g2o::EdgeSE2PointXY;
            landmark_observation->vertices()[0] = optimizer.vertex(1000);

            if (global_use_landmark_assignment_algorithm)
            {
                landmark_observation->vertices()[1] = optimizer.vertex(landmarkAssignment[i]);
            }
            else
            {
                landmark_observation->vertices()[1] = optimizer.vertex(stoi(detections[i].id));
            }

            Eigen::Matrix2d inf_matrix = utils::covariance_arrayX9_to_matrix2d(detections[i].covariance).inverse();

            landmark_observation->setMeasurement(g2o::Vector2(detections[i].position.x, detections[i].position.y));
            landmark_observation->setInformation(inf_matrix);
            optimizer.addEdge(landmark_observation);
        }
        // Perform optimization
        optimizer.initializeOptimization();
        optimizer.setVerbose(true);
        optimizer.optimize(100);

        // Compute error
        g2o::VertexSE2 *vehicle_pose_estimated = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(1000));
        std::cout << " - Vehicle pose (estimated): ( ";
        std::cout << vehicle_pose_estimated->estimate().translation().transpose() << " ) | "
                  << vehicle_pose_estimated->estimate().rotation().angle() << std::endl;

        // vehicle_pose_estimated->estimate().rotation().
        //  End timer
        double timeSpent = utils::tic();
        cout << "-----> Time spent in Graph Localitation Algorithm: " << timeSpent << " micros" << endl
             << endl;

        vehicle_pose = vehicle_pose_estimated->estimate();

        results.time_spent.push_back(timeSpent);
        results.car_position_estimation.position = geometry_msgs::build<geometry_msgs::msg::Point>()
                                                       .x(vehicle_pose[0])
                                                       .y(vehicle_pose[1])
                                                       .z(vehicle_pose.rotation().angle());
        publisher->publish(results);

        cout << "-----------------" << endl
             << endl;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto main_node = std::make_shared<LandmarksGraphLocationNode>();

    main_node->wait_for_initial_pose();
    rclcpp::spin(main_node);

    rclcpp::shutdown();
    return 0;
}