#include <ur16_repair/repair/plane_geometry.hpp>
#include <iostream>

using namespace plane_geometry;


Vector plane_geometry::pose2vector(const Pose &pose) {
    return Vector(pose.position.x, pose.position.y, pose.position.z);
}

Point plane_geometry::vector2point(const Vector &v) {
    Point p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    return p;
}

std::vector<Vector> plane_geometry::poses2vectors(const PoseArray &poses) {
    std::vector<Vector> v;
    for (const auto &pose : poses.poses) {
        v.push_back(plane_geometry::pose2vector(pose));
    }
    return v;
}


Plane::Plane(const Vector &c1, const Vector &c2, const Vector &c3, const Vector &c4) {
    corners_.resize(4);
    corners_[0] = c1;
    corners_[1] = c2;
    corners_[2] = c3;
    corners_[3] = c4;
}

std::vector<Vector> Plane::corners() const {return corners_;}

Vector Plane::vertex(int i) const {return corners_[i];}

Vector Plane::centrroid() const {
    return 0.25*(corners_[0] + corners_[1] + corners_[2] + corners_[3]);
}

Vector Plane::normal() const {
    Vector u = corners_[1] - corners_[0];
    Vector v = corners_[2] - corners_[0];
    Vector n = u.cross(v).normalized();
    return n; 
}

Vector Plane::projectPointOntoPlane (const Vector &point) const {
    Vector plane_pt = corners_[0];
    Vector n = normal();
    double d = (point - plane_pt).dot(n);
    return point - d * n;
}
Quat Plane::quat() const {
    Vector n = normal();
    Quat q; 
    return q.setFromTwoVectors(Vector::UnitZ(), n);
}

double Plane::lenght() const {
    return (corners_[1] - corners_[0]).norm();
}

double Plane::widht() const {
    return (corners_[2] - corners_[1]).norm();
}

void Plane::print() {
    std::cout << "Plane: {";
    for (const auto &corner : corners_) 
        std::cout << corner << ",";
    std::cout << "}" << std::endl;
}