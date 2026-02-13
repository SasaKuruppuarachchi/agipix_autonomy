#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>

class MapSaverNode : public rclcpp::Node {
public:
    MapSaverNode()
    : rclcpp::Node("save_map_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/occupancy_map/voxel_map",
            rclcpp::SensorDataQoS(),
            std::bind(&MapSaverNode::mapSaverCB, this, std::placeholders::_1));
    }

private:
    void mapSaverCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg){
        RCLCPP_INFO(this->get_logger(), "[Map Saver]: Latest map message obtained.");

        pcl::PCLPointCloud2 pclCloud2;
        pcl::PointCloud<pcl::PointXYZ> pclCloud;

        pcl_conversions::toPCL(*cloud_msg, pclCloud2);
        pcl::fromPCLPointCloud2(pclCloud2, pclCloud);

        const std::string file_name = "static_map.pcd";
        const int result = pcl::io::savePCDFileASCII(file_name, pclCloud);
        if (result == 0){
            RCLCPP_INFO(this->get_logger(), "[Map Saver]: Saved %zu data points to %s in the current directory.", pclCloud.size(), file_name.c_str());
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Map Saver]: Failed to save %s (error code: %d).", file_name.c_str(), result);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}