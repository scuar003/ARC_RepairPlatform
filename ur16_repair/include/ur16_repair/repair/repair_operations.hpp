#ifndef REPAIR_OPERATIONS_HPP
#define REPAIR_OPERATIONS_HPP

#include <string>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
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
#include <cmath>
#include <Eigen/Dense>
#include <functional>
#include <repair_interface/msg/eigen_msg.hpp>

namespace {
    inline double deg2rad(double d) {return d*M_PI/180.0;}
}

namespace repairs {

using namespace ur_rtde;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseArray = geometry_msgs::msg::PoseArray;
using Vec3 = Eigen::Vector3d;
using CusEigen = repair_interface::msg::EigenMsg;
using ToolRequestFn = std::function<bool(const std::string&)>;

class RepairOperations {
    public:
        RepairOperations();
        RepairOperations(std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string& target_frame, ToolRequestFn tool_request);
        PoseArray detect(const PointCloud2::SharedPtr cloud_msg);
        void scanEnv(const std::string &rb_ip);
        void moveHome(const std::string &rb_ip);
        
        
        void grindArea(const std::string& rb_ip, const std::vector<CusEigen> &area);
        void getGrinder(const std::string& rb_ip);
        void returnGrinder(const std::string& rb_ip);

        ///------------------------------ Cleaning operations --------------------------------------///



        void clean(const std::string &rb_ip, const std::vector<CusEigen> &area);

        ///------------------------------------------------------------------------------------------///

    private:
        void connect(const std::string &robot_ip);

        void printPath(const std::vector<std::vector<double>> &wp) const;
        static Vec3 normalFromCorners(const std::vector<Vec3> &c);
        static Vec3 rpyFromNormal(const Vec3 &n);
        /* path generation & execution */
        std::vector<std::vector<double>> buildPath(const std::vector<Vec3> &c, 
                                                    double grid, double lift,
                                                    int layers, double step_down);
        void executePath(const std::vector<std::vector<double>> &wp, double acc, double vel);
        

        double rb_acc = 0.08;
        double rb_vel = 0.08;
        std::unique_ptr<RTDEControlInterface> robot_;
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        ToolRequestFn tool_request_;

};

}


#endif