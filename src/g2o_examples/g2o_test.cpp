/* Example from http://wangxinliu.com/slam/optimization/research&study/g2o-4 */

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <iostream>
#include <stdint.h>
#include <vector>
#include <cmath>

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

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
// Remove "-Wpedantic with tf2/utils.h to avoid warnings about extra ';'"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <tf2/utils.h>
#pragma GCC diagnostic pop

namespace g2o_test
{

    // Creates and returns a quaternion msg from a given yaw angle
    template <typename T>
    geometry_msgs::msg::Quaternion quaternion_msg_from_yaw(T yaw)
    {
        geometry_msgs::msg::Quaternion q;
        tf2::Quaternion tf_q;

        tf_q.setRPY(0.0, 0.0, yaw);

        q.x = tf_q.x();
        q.y = tf_q.y();
        q.z = tf_q.z();
        q.w = tf_q.w();

        return q;
    }

    // Extracts yaw angle from quaternion msg
    template <typename T>
    T yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        return tf2::getYaw(tf_q);
    }

    // Converts degrees to radians in compile time
    template <typename T>
    constexpr double deg_to_rad(const T &deg) { return (M_PI * deg / 180.0); }

    // Smooth error function f(x) = x / (k + abs(x)), where k is the smoothness parameter
    // template<typename T>
    // T smooth(T x, T smoothness);
    double smooth(const double &x, const double &smoothness)
    {
        return (x / (smoothness + std::abs(x)));
    }

    // Angle normalization to [0-360] range
    template <typename T>
    T norm_angle(const T &angle)
    {
        return angle < 0 ? angle + deg_to_rad(360.0) : angle;
    }

} // Namespace g2o_test

static double uniform_rand(double lowerBndr, double upperBndr)
{
    return lowerBndr + ((double)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
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

int uniform(int from, int to)
{
    return static_cast<int>(uniform_rand(from, to));
}

double uniform()
{
    return uniform_rand(0., 1.);
}

double gaussian(double sigma)
{
    return gauss_rand(0., sigma);
}

std::vector<Eigen::Vector2d> generate_landmark_points()
{
    std::vector<Eigen::Vector2d> poses{
        Eigen::Vector2d{5.0, 40.0},
        Eigen::Vector2d{-40.0, 50.0},
        Eigen::Vector2d{55.0, 25.0},
        Eigen::Vector2d{-65.0, 55.0},
        Eigen::Vector2d{85.0, 60.0}};

    return poses;
}

void compute_landmark_measurement(const g2o::SE2 &pose, const Eigen::Vector2d &landmark,
                                  Eigen::Vector2d &measurement, Eigen::Matrix2d &inf_matrix)
{
    // Compute the perfect measurement
    Eigen::Vector2d true_measurement = pose.inverse() * landmark;
    // Add gaussian noise
    measurement = true_measurement + Eigen::Vector2d{gaussian(0.1), gaussian(0.1)};
    // Fill the information matrix
    inf_matrix = Eigen::Matrix2d::Identity() * 10;
}

int main()
{
    std::vector<Eigen::Vector2d> landmark_points = generate_landmark_points();

    std::cout << "Landmark points (ground truth): " << std::endl;
    for (const auto &p : landmark_points)
    {
        std::cout << "* " << p.transpose() << std::endl;
    }

    g2o::SE2 vehicle_pose_gt(10.0, 11.0, g2o_test::deg_to_rad<double>(0.0));
    g2o::SE2 vehicle_pose_initial_guess(7.7, 6.0, g2o_test::deg_to_rad<double>(15.0));
    std::cout << "Vehicle pose (ground truth):" << std::endl;
    std::cout << vehicle_pose_gt.translation().transpose() << " | " << vehicle_pose_gt.rotation().angle() << std::endl;
    std::cout << "Vehicle pose (initial guess):" << std::endl;
    std::cout << vehicle_pose_initial_guess.translation().transpose() << " | " << vehicle_pose_initial_guess.rotation().angle() << std::endl;

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
    for (size_t i = 0; i < landmark_points.size(); i++)
    {
        g2o::VertexPointXY *landmark = new g2o::VertexPointXY;
        landmark->setId(i);
        landmark->setFixed(true);
        landmark->setEstimate(landmark_points[i]);
        optimizer.addVertex(landmark);
    }

    // Set vertex from vehicle pose (initial guess)
    g2o::VertexSE2 *vehicle_pose_vertex = new g2o::VertexSE2;
    vehicle_pose_vertex->setId(1000);
    vehicle_pose_vertex->setEstimate(vehicle_pose_initial_guess);
    optimizer.addVertex(vehicle_pose_vertex);

    // Set measurements (graph edges)
    for (size_t i = 0; i < landmark_points.size(); i++)
    {
        Eigen::Vector2d landmark = landmark_points[i];

        g2o::EdgeSE2PointXY *landmark_observation = new g2o::EdgeSE2PointXY;
        landmark_observation->vertices()[0] = optimizer.vertex(1000);
        landmark_observation->vertices()[1] = optimizer.vertex(i);

        Eigen::Vector2d measurement;
        Eigen::Matrix2d inf_matrix;
        compute_landmark_measurement(vehicle_pose_gt, landmark, measurement, inf_matrix);

        landmark_observation->setMeasurement(measurement);
        landmark_observation->setInformation(inf_matrix);
        optimizer.addEdge(landmark_observation);
    }
    // Perform optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(100);

    // Compute error
    g2o::VertexSE2 *vehicle_pose = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(1000));
    std::cout << "Vehicle pose (estimated):" << std::endl;
    std::cout << vehicle_pose->estimate().translation().transpose() << " | "
              << vehicle_pose->estimate().rotation().angle() << std::endl;

    return 0;
}
