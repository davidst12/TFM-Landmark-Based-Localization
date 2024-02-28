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

class MainNode : public rclcpp::Node
{
    public:
    // Constructor
    MainNode(): Node("main_node", rclcpp::NodeOptions())
    {
        subscription_ = this->create_subscription<detection_msgs::msg::DetectionArray>("/fake_pole_detection", 1, std::bind(&MainNode::detectionsCallback, this, _1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/carla/ego_vehicle/gnss", 1, std::bind(&MainNode::gnssCallback, this, _1));
        publisher = this->create_publisher<tfm_landmark_based_localization_package::msg::Results>("/results", 1);


        landmarkPoses = utils::getLandmarkPoses();

        // Global parameters
        declare_parameter("use_landmark_assignment_algorithm", false);
        declare_parameter("gps_noise_error_sigma", 0.0);
        global_use_landmark_assignment_algorithm = get_parameter("use_landmark_assignment_algorithm").as_bool();
        global_gps_noise_error_sigma = get_parameter("gps_noise_error_sigma").as_double();
        cout << "Global parameters" << endl;
        cout << " - Use_landmark_assignment_algorithm -> " << global_use_landmark_assignment_algorithm << endl;
        cout << " - Gps_noise_error_sigma -> " << global_gps_noise_error_sigma << endl;

        recoveryCounter = 0;
    }

    // Wait for initial pose from user
    void waitForInitialPose(){
        cout << "Please, enter initial pose X and Y of the car" << endl;
        string poseX;
        string poseY;
        cin >> poseX;
        cin >> poseY;
        cout << "Initial car pose: ( " << poseX << " , " << poseY << " )" << endl;
        vehiclePose = g2o::SE2(stod(poseX), stod(poseY), 0.0);
    }

    private:

    rclcpp::Subscription<detection_msgs::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription2_;
    rclcpp::Publisher<tfm_landmark_based_localization_package::msg::Results>::SharedPtr publisher;

    std::vector<utils::LandmarkObject> landmarkPoses;
    std::vector<detection_msgs::msg::Detection> detections;

    g2o::SE2 vehiclePose;
    g2o::SE2 gnssVehiclePose;
    bool global_use_landmark_assignment_algorithm;
    double global_gps_noise_error_sigma;

    tfm_landmark_based_localization_package::msg::Results results;
    int recoveryCounter;


    void cleanResults(){
        results.car_position_estimation = geometry_msgs::msg::Pose();
        results.detections_assignment = vector<int>();
        results.time_spent = vector<double>();
        results.cost = 0.0;
    }

    // Callback of "/fake_pole_detection" topic
    void detectionsCallback(const detection_msgs::msg::DetectionArray & detectionsList) {
        cout << " - Detections received: " << detectionsList.detections.size() << endl;

        detections.clear();
        cleanResults();

        for (unsigned int i = 0; i<detectionsList.detections.size(); i++){
            detections.push_back(detectionsList.detections[i]);
        }

        results.header = detectionsList.header;

        estimateLocalitation();
    }

    // Callback of "/carla/ego_vehicle/gnss" topic
    void gnssCallback(const sensor_msgs::msg::NavSatFix & gnssValue) {
        
        if(recoveryCounter > 1){
            double lat = gnssValue.latitude;
            double lon = gnssValue.longitude;

            double y = 6371000 * tan(lat*3.1415/180) + errors::gaussian(global_gps_noise_error_sigma); // 1 o 2 metros
            double x = 6371000 * tan(lon*3.1415/180) + errors::gaussian(global_gps_noise_error_sigma);

            gnssVehiclePose = g2o::SE2(x, y, 0.0);
            //cout << "x: " << x << " , y: " << y << endl;
        }
    }

    // Distance: car -> landmark_detected
    double calculateDistance(Eigen::Vector2d landmarkPose, g2o::SE2 poseCar, detection_msgs::msg::Detection measure){

        double l1 = pow(landmarkPose[0]-(poseCar[0]+measure.position.x), 2);
        double l2 = pow(landmarkPose[1]-(poseCar[1]+measure.position.y), 2);

        return sqrt(l1+l2);
    }

    // Association Algorithm
    vector<int> assignmentLandmarkAlgorithm(){
        cout << "-----------------" << endl << endl;
        cout << "Landmark - Measure association algorithm (Hungarian Method)" << endl;

        // Start timer
        utils::tic();

        // Fill Cost Matrix
        vector< vector<double> > costMatrix(landmarkPoses.size(), vector<double>(detections.size(), 0.0));

        for(unsigned int i = 0; i < landmarkPoses.size(); i++){
            for(unsigned int j = 0; j < detections.size(); j++){
                double a = calculateDistance(landmarkPoses[i].getPose(), vehiclePose, detections[j]);
                costMatrix[i][j] = a;
            }
        }

        // Run algorithm
        HungarianAlgorithm HungAlgo;
        vector<int> assignment;
        vector<int> finalAssignment;

        double cost = HungAlgo.Solve(costMatrix, assignment);

        // Associate landmark - measure
        for (unsigned int landmarkEntry = 0; landmarkEntry < costMatrix.size(); landmarkEntry++){
            int measureEntry = assignment[landmarkEntry];
            if(measureEntry != -1){
                finalAssignment.push_back(landmarkEntry);
                std::cout << " - Landmark [" << landmarkEntry << "] -> Measure " << measureEntry << endl;
            }
        }
        std::cout << " - Cost: " << cost << std::endl;

        if(cost > 50){
            recoveryCounter++;
        }else{
            recoveryCounter = 0;
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
    void estimateLocalitation(){

        // If recovery is true -> vehiclePose lo cojo del GNSS
        results.recovery_applied = false;
        if(recoveryCounter == 3){
            recoveryCounter = 0;
            vehiclePose = gnssVehiclePose;
            results.recovery_applied = true;
        }

        vector<int> landmarkAssignment;
        if(global_use_landmark_assignment_algorithm){
            landmarkAssignment = assignmentLandmarkAlgorithm();
            //cout << "Detections == Assignment: " << (landmarkAssignment.size() == detections.size()) << endl;
        }

        cout << endl << "Graph Localitation Algorithm" << endl;

        // Start timer
        utils::tic();

        // Setup optimizer algorithm and solver
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(true);

        using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >;
        using SlamLinearSolver = g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType>;
        auto linearSolver = std::make_unique<SlamLinearSolver>();
        linearSolver->setBlockOrdering(false);
        g2o::OptimizationAlgorithmLevenberg* solver =
            new g2o::OptimizationAlgorithmLevenberg(std::make_unique<SlamBlockSolver>(std::move(linearSolver)));
        optimizer.setAlgorithm(solver);


        // Set fixed vertices from map of landmarks
        for (size_t i = 0; i < landmarkPoses.size(); i++)
        {
            g2o::VertexPointXY* landmark = new g2o::VertexPointXY;
            landmark->setId(i);
            landmark->setFixed(true);
            landmark->setEstimate(landmarkPoses[i].getPose());
            optimizer.addVertex(landmark);
        }

        // Set vertex from vehicle pose (initial guess)
        g2o::VertexSE2* vehicle_pose_vertex = new g2o::VertexSE2;
        vehicle_pose_vertex->setId(1000);
        vehicle_pose_vertex->setEstimate(vehiclePose);
        optimizer.addVertex(vehicle_pose_vertex);

        // cout << "vehicle pose = " << vehiclePose[0] << " , " << vehiclePose[1] << endl;

        // Set detections (graph edges)
        for (size_t i = 0; i < detections.size(); i++)
        {

            g2o::EdgeSE2PointXY* landmark_observation = new g2o::EdgeSE2PointXY;
            landmark_observation->vertices()[0] = optimizer.vertex(1000);

            if(global_use_landmark_assignment_algorithm){
                landmark_observation->vertices()[1] = optimizer.vertex(landmarkAssignment[i]);
            }else{
                landmark_observation->vertices()[1] = optimizer.vertex(stoi(detections[i].id));
            }

            //Eigen::Vector2d measurement;
            //Eigen::Matrix2d inf_matrix = Eigen::Matrix2d::Identity() * detections[i].covariance[0];
            Eigen::Matrix2d inf_matrix = utils::covarianceArrayX9toMatrix2d(detections[i].covariance).inverse();

            landmark_observation->setMeasurement(g2o::Vector2(detections[i].position.x, detections[i].position.y));
            landmark_observation->setInformation(inf_matrix);
            optimizer.addEdge(landmark_observation);
        }
        // Perform optimization
        optimizer.initializeOptimization();
        optimizer.setVerbose(true);
        optimizer.optimize(100);

        // Compute error
        g2o::VertexSE2* vehicle_pose = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(1000));
        std::cout << " - Vehicle pose (estimated): ( ";
        std::cout << vehicle_pose->estimate().translation().transpose() << " ) | "
                    << vehicle_pose->estimate().rotation().angle() << std::endl;

        // End timer
        double timeSpent = utils::tic();
        cout << "-----> Time spent in Graph Localitation Algorithm: " << timeSpent << " micros" << endl << endl;

        vehiclePose = vehicle_pose->estimate();
        //cout << "Pose estimation check: " << vehiclePose.toVector() << endl;

        results.time_spent.push_back(timeSpent);
        results.car_position_estimation.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(vehiclePose[0]).y(vehiclePose[1]).z(0.0);
        publisher->publish(results);

        cout << "-----------------" << endl << endl;

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto mainNode = std::make_shared<MainNode>();

    mainNode->waitForInitialPose();
    rclcpp::spin(mainNode);
    
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}