#pragma once
#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace plane_geometry {
    using Vector = Eigen::Vector3d;
    using Quat = Eigen::Quaterniond;
    using Pose = geometry_msgs::msg::Pose;
    using Point = geometry_msgs::msg::Point;
    using PoseArray = geometry_msgs::msg::PoseArray;


    Vector pose2vector (const Pose &pose);
    Point vector2point(const Vector &v);
    std::vector<Vector> poses2vectors(const PoseArray &poses);



    class Plane {
        public:
            Plane(const Vector &c1, const Vector &c2, const Vector &c3, const Vector &c4);
            
            std::vector<Vector> corners() const; 
            Vector vertex (int i) const;
            Vector centrroid() const;
            Vector normal() const ;
            Vector projectPointOntoPlane(const Vector& point) const;
            Quat quat() const;
            double lenght() const;
            double widht()const;
            void print();  
            
            
        private:
            std::vector<Vector> corners_;
            Vector size_;

    };

}