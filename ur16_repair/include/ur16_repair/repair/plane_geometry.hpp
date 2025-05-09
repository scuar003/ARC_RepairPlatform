#pragma once
#include <Eigen/Dense>
#include <vector>

namespace u16_geometry {
    using Vector = Eigen::Vector3d;
    using Quat = Eigen::Quaterniond;

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