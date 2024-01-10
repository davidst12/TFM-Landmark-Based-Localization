/* Example from http://wangxinliu.com/slam/optimization/research&study/g2o-4 */

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <iostream>
#include <stdint.h>
#include <vector>
#include <math.h>
#include <unordered_set>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/edge_xyz_prior.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include "edge_se3exp_pointxyz_prior.h"


static double uniform_rand(double lowerBndr, double upperBndr)
{
    return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gauss_rand(double mean, double sigma)
{
    double x, y, r2;
    do
    {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);

    return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int uniform(int from, int to){
    return static_cast<int>(uniform_rand(from, to));
}

double uniform(){
    return uniform_rand(0., 1.);
}

double gaussian(double sigma){
    return gauss_rand(0., sigma);
}

struct CameraPose{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};

std::vector<CameraPose> generate_camera_poses()
{
    std::vector<CameraPose> poses{};
    for (int i = 0; i < 10; i++)
    {
        poses.push_back(CameraPose{Eigen::MatrixXd::Identity(3,3), Eigen::Vector3d{0, 0, (double)i}});
    }
    return poses;
}

double compute_pose_error(std::vector<g2o::VertexSE3Expmap*> measure,
                          std::vector<CameraPose> ground_truth)
{
    double error = 0;
    for (size_t i = 0; i < measure.size(); i++)
    {        
        error += (((g2o::SE3Quat)(measure[i]->estimate())).translation() - ground_truth[i].t).norm();
    }
    return error / measure.size();
}


int main()
{
    std::vector<CameraPose> poses = generate_camera_poses();

    std::cout << "Camera poses (ground truth): " << std::endl;
    for (auto pose : poses)
    {
        std::cout << "* " << pose.t.transpose() << std::endl;
    }

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    // std::unique_ptr<g2o::BlockSolverPL<6, 1>::LinearSolverType> linearSolver;
    // bool DENSE = true;
    // if (DENSE) {
    //   linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_1::PoseMatrixType>>();
    // } else {
    //   linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverPL<6, 1>::PoseMatrixType>>();
    // }

    auto linearSolverType = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverPL<6, 1>::PoseMatrixType>>();

    auto solver = g2o::make_unique<g2o::BlockSolverPL<6, 1>>(std::move(linearSolverType));

    g2o::OptimizationAlgorithmLevenberg* optimaAlgorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(solver));

    optimizer.setAlgorithm(optimaAlgorithm);

    std::vector<g2o::VertexSE3Expmap*> se3_vertices;
    int vertex_id = 0;
    for (auto pose : poses)
    {
        std::cout << "-------------------------------------" << std::endl;
        // Add the pose vertices
        g2o::SE3Quat curr_pose(pose.R, pose.t);

        std::cout << "Current pose:\n" << curr_pose.translation().transpose() << std::endl;
        g2o::VertexSE3Expmap* vertex = new g2o::VertexSE3Expmap();
        vertex->setId(vertex_id);
        vertex->setFixed(false);

        curr_pose.setTranslation(curr_pose.translation() + 2.0f * Eigen::Vector3d(uniform(),
                                                                                  uniform(),
                                                                                  uniform()));
        std::cout << "Current pose translated:\n" << curr_pose.translation().transpose() << std::endl;
        vertex->setEstimate(curr_pose);
        std::cout << "vertex: ";
        vertex->write(std::cout);
        std::cout << std::endl;

        se3_vertices.push_back(vertex);
        optimizer.addVertex(vertex);
        vertex_id++;

        g2o::EdgeSE3ExpXYZPointPrior* gps_constrains = new g2o::EdgeSE3ExpXYZPointPrior();
        gps_constrains->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex));

        std::cout << "edge: ";
        gps_constrains->write(std::cout);
        std::cout << std::endl;

        Eigen::Vector3d noisyMeasurement = pose.t + 0.1f * Eigen::Vector3d(uniform(),
                                                                           uniform(),
                                                                           uniform());

        std::cout << "noisy measurement: " << noisyMeasurement.transpose() << std::endl;

        gps_constrains->setMeasurement(noisyMeasurement);
        gps_constrains->setInformation(Eigen::Matrix3d::Identity());

        std::cout << "edge 3: ";
        gps_constrains->write(std::cout);
        std::cout << std::endl;

        optimizer.addEdge(gps_constrains);
    }

    double pose_before = compute_pose_error(se3_vertices, poses);

    optimizer.initializeOptimization();
    optimizer.setVerbose(false);
    optimizer.optimize(100);

    double pose_after = compute_pose_error(se3_vertices, poses);
    std::cout << "Error before, after: " << pose_before << ", " << pose_after << std::endl;

    return 0;
}
