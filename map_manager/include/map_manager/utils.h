/*
    File: utils.h
    -----------------
    Autonomous Flight utils and miscs. 
*/ 

#ifndef AUTOFLIGHTUTILS_H
#define AUTOFLIGHTUTILS_H
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>


namespace AutoFlight{
    const double PI_const = 3.1415926;
    struct pose{
        double x;
        double y;
        double z;
        double yaw;
        pose(){
            x = 0; y = 0; z = 0; yaw = 0;
        }
        pose(double _x, double _y, double _z){
            x = _x; y = _y; z = _z; yaw = 0;
        }   

        pose(double _x, double _y, double _z, double _yaw){
            x = _x; y = _y; z = _z; yaw = _yaw;
        }
    };

    struct velocity{
        double vx;
        double vy;
        double vz;
        velocity(){
            vx = 0; vy = 0; vz = 0;
        }
        velocity(double _vx, double _vy, double _vz){
            vx = _vx; vy = _vy; vz = _vz;
        }
    };

    struct circleData{
		double x;
        double y;
        double z;
        double vx;
        double vy;
        double vz;
        double ax;
        double ay;
        double az;
        double yaw;
        double theta;
        double radius;
        double velocity;
		int rate;
        double cycle_s;
        //rclcpp::GenericRate r;
		double theta_start;
        double theta_end;
        double step;
        int steps;
        int circle;
        int terminate;
        int index;
		rclcpp::Time startTime;
		rclcpp::Time endTime1, endTime2, endTime3;
	};

    struct state{
        pose p;
        velocity v;
        state(){
            p = pose(0, 0, 0, 0);
            v = velocity(0, 0, 0);
        }
        state(const pose& _p, const velocity& _v){
            p = _p; v = _v;
        }
        state(double _px, double _py, double _pz, double _yaw, double _vx, double _vy, double _vz){
            p = pose(_px, _py, _pz, _yaw);
            v = velocity(_vx, _vy, _vz);
        }
    };

    inline std::ostream &operator<<(std::ostream &os, pose& pose){
        os << "pose: (" << pose.x << " " << pose.y << " " << pose.z << " " << pose.yaw << ")";
        return os;
    }
    inline geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        if (yaw > PI_const){
            yaw = yaw - 2*PI_const;
        }
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    inline float mapValue(float value, float in_min, float in_max, float out_min, float out_max) {
        float normalized = (value - in_min) / (in_max - in_min) * 2.0f - 1.0f;
        float transformed = normalized * std::abs(normalized); // Quadratic effect: x * |x|
        return (transformed + 1.0f) * (out_max - out_min) / 2.0f + out_min;
    }

    inline geometry_msgs::msg::Quaternion eigenQuaternionToMsg(const Eigen::Quaternionf& eigen_quat) {
        geometry_msgs::msg::Quaternion ros_quat;

        ros_quat.x = eigen_quat.x();
        ros_quat.y = eigen_quat.y();
        ros_quat.z = eigen_quat.z();
        ros_quat.w = eigen_quat.w();

        return ros_quat;
    }

    inline double rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quat){
        // return is [0, 2pi]
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    inline double rpy_from_quaternion(const Eigen::Quaternionf& quate){
        // return is [0, 2pi]
        tf2::Quaternion tf_quat;
        geometry_msgs::msg::Quaternion quat = eigenQuaternionToMsg(quate);
        tf2::convert(quat, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    inline void rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quat, double &roll, double &pitch, double &yaw){
        tf2::Quaternion tf_quat;
        tf2::convert(quat, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    inline void rpy_from_quaternion(const Eigen::Quaternionf& quate, double &roll, double &pitch, double &yaw){
        tf2::Quaternion tf_quat;
        geometry_msgs::msg::Quaternion quat = eigenQuaternionToMsg(quate);
        tf2::convert(quat, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    }

    inline double getPoseDistance(const pose& p1, const pose& p2){
        return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));    

    }

    inline double getPoseDistance(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2){
        return sqrt(pow((p1.position.x - p2.position.x),2) + pow((p1.position.y - p2.position.y),2) + pow((p1.position.z - p2.position.z),2));    
    }

    inline double getPoseDistance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2){
        return sqrt(pow((p1.pose.position.x - p2.pose.position.x),2) + pow((p1.pose.position.y - p2.pose.position.y),2) + pow((p1.pose.position.z - p2.pose.position.z),2));    
    }

    inline double getAngleDiff(double angle1, double angle2){
        double delta = std::abs(angle2 - angle1);
        if (delta > PI_const){
            delta = 2 * PI_const - delta;
        }
        return delta;
    }

    inline double getYawDistance(const pose& pStart, const pose& pTarget){
        double yaw1 = pStart.yaw;
        double yaw2 = pTarget.yaw;
        double delta = std::abs(yaw2 - yaw1);
        if (delta > PI_const){
            delta = 2 * PI_const - delta;
        }
        return delta;
    }
    
    // Helper Function: Random Number
    inline double randomNumber(double min, double max){
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> distribution(min, max);
        return distribution(mt);
    }

    inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
        Eigen::Matrix3d rotmat;
        rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
            2 * q(0) * q(2) + 2 * q(1) * q(3),

            2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
            2 * q(2) * q(3) - 2 * q(0) * q(1),

            2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
            q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
        return rotmat;
    }

    inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
        Eigen::Vector4d quat;
        double tr = R.trace();
        if (tr > 0.0){
            double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
            quat(0) = 0.25 * S;
            quat(1) = (R(2, 1) - R(1, 2)) / S;
            quat(2) = (R(0, 2) - R(2, 0)) / S;
            quat(3) = (R(1, 0) - R(0, 1)) / S;
        } 
        else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))){
            double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
            quat(0) = (R(2, 1) - R(1, 2)) / S;
            quat(1) = 0.25 * S;
            quat(2) = (R(0, 1) + R(1, 0)) / S;
            quat(3) = (R(0, 2) + R(2, 0)) / S;
        } 
        else if (R(1, 1) > R(2, 2)){
            double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
            quat(0) = (R(0, 2) - R(2, 0)) / S;
            quat(1) = (R(0, 1) + R(1, 0)) / S;
            quat(2) = 0.25 * S;
            quat(3) = (R(1, 2) + R(2, 1)) / S;
        } 
        else{
            double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
            quat(0) = (R(1, 0) - R(0, 1)) / S;
            quat(1) = (R(0, 2) + R(2, 0)) / S;
            quat(2) = (R(1, 2) + R(2, 1)) / S;
            quat(3) = 0.25 * S;
        }
        return quat;
    }
    template <class T>
    bool parse_param(const std::string &param_name, T &param_dest, const std::shared_ptr<rclcpp::Node>& node)
    {
        // firstly, the parameter has to be specified (together with its type), which can throw an exception
        if (!node->has_parameter(param_name)){
            try
            {
                node->declare_parameter<T>(param_name, T{}); // for Galactic and newer, the type has to be specified here
            }
            catch (const std::exception& e)
            {
                // this can happen if (see http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1node->html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb):
                // * parameter has already been declared              (rclcpp::exceptions::ParameterAlreadyDeclaredException)
                // * parameter name is invalid                        (rclcpp::exceptions::InvalidParametersException)
                // * initial value fails to be set                    (rclcpp::exceptions::InvalidParameterValueException, not sure what exactly this means)
                // * type of the default value or override is wrong   (rclcpp::exceptions::InvalidParameterTypeException, the most common one)
                RCLCPP_ERROR_STREAM(node->get_logger(), "Could not load param '" << param_name << "': " << e.what());
                return false;
            }
        }
        else{
            RCLCPP_WARN(node->get_logger(), "Parameter '%s' already declared. Skipping declaration.", param_name.c_str());
        }
    
        // then we can attempt to load its value from the server
        if (node->get_parameter(param_name, param_dest))
        {
            
            // Handle logging for std::vector separately
            if constexpr (std::is_same<T, std::vector<double>>::value)
            {
                std::ostringstream oss;
                oss << "[";
                for (size_t i = 0; i < param_dest.size(); ++i)
                {
                    oss << param_dest[i];
                    if (i < param_dest.size() - 1)
                        oss << ", ";
                }
                oss << "]";
                RCLCPP_INFO_STREAM(node->get_logger(), "Loaded* '" << param_name << "' = " << oss.str());
            }
            else
            {
                RCLCPP_INFO_STREAM(node->get_logger(), param_dest );
                RCLCPP_INFO_STREAM(node->get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
            }
            
            return true;
        }
        else
        {
        // this branch should never happen since we *just* declared the parameter
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not load param '" << param_name << "': Not declared!");
        return false;
        }
    }
    
    template <class T>
    T parse_param2(const std::string &param_name, bool& ok_out, rclcpp::Node& node)
    {
        T out;
        ok_out = parse_param(param_name, out, node);
        return out;
    }

    template<typename T>
    inline std::string to_string(const T & value) {
        return std::to_string(value);
    }

    inline std::string to_string(const std::string & value) {
        return value;
    }

    inline std::string to_string(const std::vector<double>& vec) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            oss << vec[i];
            if (i != vec.size() - 1) oss << ", ";
        }
        oss << "]";
        return oss.str();
    }

    template<typename T>
    inline bool parse_param(const std::shared_ptr<rclcpp::Node> & node, const std::string & param_name, T & destination_var)
    {
        if (!node->has_parameter(param_name)) {
            node->declare_parameter<T>(param_name);
        }
        else {
            RCLCPP_WARN(node->get_logger(), "Parameter '%s' already declared. Skipping declaration.", param_name.c_str());
        }

        if (node->get_parameter(param_name).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            RCLCPP_WARN(node->get_logger(), "Parameter '%s' not set.", param_name.c_str());
            return false;
        }

        try {
            destination_var = node->get_parameter(param_name).get_value<T>();
            RCLCPP_INFO(node->get_logger(), "Loaded parameter '%s': value = %s",
                        param_name.c_str(), to_string(destination_var).c_str());
            return true;
        } catch (const std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "Failed to get parameter '%s': %s", param_name.c_str(), e.what());
            return false;
        }
    }
}

#endif