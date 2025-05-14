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

#include <memory>
#include <vector>
#include <Eigen/Dense>


namespace repairs {

using namespace ur_rtde;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseArray = geometry_msgs::msg::PoseArray;
using Vec3 = Eigen::Vector3d;


class RepairOperations {
    public:
        RepairOperations();
        RepairOperations(std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string& target_frame);
        PoseArray detect(const PointCloud2::SharedPtr cloud_msg);
        void scanEnv(const std::string &rb_ip);
        void moveHome(const std::string &rb_ip);
        
        
        void grindArea(const std::string& rb_ip, const PoseArray &area);
        void getGrinder(const std::string& rb_ip);
        void returnGrinder(const std::string& rb_ip);

    private:
        void connect(const std::string &robot_ip);
        static Vec3 normalFromCorners(const std::vector<Vec3> &c);
        static Vec3 rpyFromNormal(const Vec3 &n);

    /* path generation & execution */
        std::vector<std::array<double,6>> buildPath(const std::vector<Vec3> &c, 
                                                    double grid, double lift,
                                                    int layers, double step_down);
        void executePath(const std::vector<std::array<double,6>> &wp, double acc, double vel);
        
        std::unique_ptr<RTDEControlInterface> robot_;
        double rb_acc = 0.08;
        double rb_vel = 0.08;
        
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

};

}


#endif