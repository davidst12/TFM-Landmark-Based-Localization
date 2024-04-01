#include <string>
#include <fstream>
#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using json = nlohmann::json;

namespace utils
{
    class LandmarkObject
    {
    public:
        LandmarkObject(std::string id, double pose_x, double pose_y)
        {
            _id = id;
            _pose[0] = pose_x;
            _pose[1] = pose_y;
        }
        std::string get_id() { return _id; }
        Eigen::Vector2d get_pose() { return _pose; }

    private:
        std::string _id;
        Eigen::Vector2d _pose;
    };

    /*
     * Get landmarks from config file
     */
    std::vector<LandmarkObject> get_landmark_poses(std::string package_prefix, std::string landmarks_ground_truth_file)
    {

        std::string package_path = ament_index_cpp::get_package_prefix(package_prefix);
        std::ifstream f(package_path + landmarks_ground_truth_file);

        json data = json::parse(f);
        std::vector<std::vector<int>> l = data["landmarks"];
        std::vector<LandmarkObject> landmarks_array;

        for (unsigned int i = 0; i < l.size(); i++)
        {
            LandmarkObject landmark = LandmarkObject(std::to_string(i), l[i][0], l[i][1]);
            landmarks_array.push_back(landmark);
        }
        std::cout << "Number of landmarks: " << landmarks_array.size() << std::endl;

        return landmarks_array;
    }

    /*
     * Calculate time spent in microseconds
     */
    double tic()
    {
        static std::chrono::time_point<std::chrono::steady_clock> last_tic;
        auto new_tic = std::chrono::steady_clock::now();
        double micros = std::chrono::duration<double, std::micro>(new_tic - last_tic).count();
        last_tic = new_tic;
        return micros;
    }

    /*
     * Transform array (size = 9) into a matrix 2d (row, colum)
     */
    Eigen::Matrix2d covariance_arrayX9_to_matrix2d(std::array<double, 9UL> covariance_array)
    {
        Eigen::Matrix2d aux;
        aux(0, 0) = covariance_array[0];
        aux(0, 1) = covariance_array[1];
        aux(1, 0) = covariance_array[3];
        aux(1, 1) = covariance_array[4];

        return aux;
    }

    /*
     * Transform array (size = 9) into a matrix 3d
     */
    Eigen::Matrix3d covariance_arrayX9_to_matrix3d(std::array<double, 9UL> covariance_array)
    {
        Eigen::Matrix3d aux;
        int i = 0;
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                aux(row, col) = covariance_array[i];
                i++;
            }
        }
        return aux;
    }

    /*
     * Transform matrix 2d into array (size = 9)
     */
    std::array<double, 9UL> matrix2d_to_covariance_arrayX9(Eigen::Matrix2d covariance_matrix)
    {
        return {covariance_matrix(0, 0), covariance_matrix(0, 1), 0,
                covariance_matrix(1, 0), covariance_matrix(1, 1), 0,
                0, 0, 0};
    }

    // Converts degrees to radians in compile time
    template <typename T>
    constexpr double deg_to_rad(const T &deg) { return (M_PI * deg / 180.0); }

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

    // Extracts yaw angle from quaternion msg. Range [-pi - pi]
    template <typename T>
    T yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

}
