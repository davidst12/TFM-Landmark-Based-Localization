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
#include "tfm_landmark_based_localization_package/msg/detection_array.hpp"
#include "tfm_landmark_based_localization_package/msg/detection.hpp"

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
        subscription_ = this->create_subscription<tfm_landmark_based_localization_package::msg::DetectionArray>("/fake_pole_detection", 1, std::bind(&MainNode::detectionsCallback, this, _1));

        landmarkPoses = utils::getLandmarkPoses();

        // Global parameters
        declare_parameter("use_landmark_assignment_algorithm", false);
        global_use_landmark_assignment_algorithm = get_parameter("use_landmark_assignment_algorithm").as_bool();
        cout << "Clobal parameters" << endl;
        cout << " - Use_landmark_assignment_algorithm -> " << global_use_landmark_assignment_algorithm << endl;
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

    rclcpp::Subscription<tfm_landmark_based_localization_package::msg::DetectionArray>::SharedPtr subscription_;

    std::vector<utils::LandmarkObject> landmarkPoses;
    std::vector<tfm_landmark_based_localization_package::msg::Detection> detections;

    g2o::SE2 vehiclePose;
    bool global_use_landmark_assignment_algorithm;

    // Callback of "/fake_pole_detection" topic
    void detectionsCallback(const tfm_landmark_based_localization_package::msg::DetectionArray & detectionsList) {
        cout << " - Detections received: " << detectionsList.detections.size() << endl;

        detections.clear();

        for (unsigned int i = 0; i<detectionsList.detections.size(); i++){
            detections.push_back(detectionsList.detections[i]);
        }

        estimateLocalitation();
    }

    // Distance: car -> landmark_detected
    double calculateDistance(Eigen::Vector2d landmarkPose, g2o::SE2 poseCar, tfm_landmark_based_localization_package::msg::Detection measure){

        double l1 = pow(landmarkPose[0]-(poseCar[0]+measure.position.x), 2);
        double l2 = pow(landmarkPose[1]-(poseCar[1]+measure.position.y), 2);

        return sqrt(l1+l2);
    }

    // Association Algorithm
    vector<int> assignmentLandmarkAlgorithm(){
        cout << "-----------------" << endl << endl;
        cout << "Landmark - Measure association algorithm (Hungarian Method)" << endl;

        // Start timer
        auto ms0 = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();

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

        // End timer
        auto ms1 = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();
        cout << "-----> Time spent in Association Algorithm: " << ms1-ms0 << " ms" << endl;

        return finalAssignment;
    }

    // Location based in graph algorithm
    void estimateLocalitation(){

        vector<int> landmarkAssignment;
        if(global_use_landmark_assignment_algorithm){
            landmarkAssignment = assignmentLandmarkAlgorithm();
            //cout << "Detections == Assignment: " << (landmarkAssignment.size() == detections.size()) << endl;
        }

        cout << endl << "Graph Localitation Algorithm" << endl;

        // Start timer
        //auto ms_0 = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();
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
            Eigen::Matrix2d inf_matrix = Eigen::Matrix2d::Identity() * 10;

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
        //auto ms_1 = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();
        //auto ms_1 = utils::tic()
        //cout << "-----> Time spent in Graph Localitation Algorithm: " << ms_1-ms_0 << " ms" << endl << endl;
        cout << "-----> Time spent in Graph Localitation Algorithm: " << utils::tic() << " micros" << endl << endl;

        cout << "-----------------" << endl << endl;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto mainNode = std::make_shared<MainNode>();

    while (rclcpp::ok())
    {
        mainNode->waitForInitialPose();
        rclcpp::spin_some(mainNode);
    }
    
    rclcpp::spin(std::make_shared<MainNode>());
    rclcpp::shutdown();
    return 0;
}