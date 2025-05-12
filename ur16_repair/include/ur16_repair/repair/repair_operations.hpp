#ifndef REPAIR_OPERATIONS_HPP
#define REPAIR_OPERATIONS_HPP

#include <string>
#include <ur_rtde/rtde_control_interface.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <open3d/Open3D.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace repairs {

using namespace ur_rtde;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseArray = geometry_msgs::msg::PoseArray;


class RepairOperations {
    public:
        RepairOperations();
        RepairOperations(std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string& target_frame);
        PoseArray detect(const PointCloud2::SharedPtr cloud_msg);
        void scanEnv(const std::string&rb_ip);

    private:



        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

};

}


#endif