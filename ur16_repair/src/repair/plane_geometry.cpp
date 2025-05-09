#include <ur16_repair/repair/plane_geometry.hpp>


using namespace u16_geometry;

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
    // Vector n = u.cross(v).
    return v; // not implemented 
}