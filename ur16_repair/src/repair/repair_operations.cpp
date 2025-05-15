#include <ur16_repair/repair/repair_operations.hpp>
#include <iostream>

namespace repairs {

    RepairOperations::RepairOperations() {}

    void RepairOperations::connect(const std::string &rb_ip) {
        if(robot_ && robot_->isConnected()) return;
        std::cout << "Connecting to robot at: " << rb_ip << std::endl;
        robot_ = std::make_unique<RTDEControlInterface>(rb_ip);
    }

    void RepairOperations::scanEnv(const std::string &rb_ip){
        connect(rb_ip);
        //Start
        moveHome(rb_ip);
        //right
        robot_->moveJ({-3.25, -0.94, -1.33, -1.65, 1.55, 0.0}, rb_acc, rb_vel);
        //left
        robot_->moveJ({0.83, -0.94, -1.33, -1.65, 1.55, 0.0}, rb_acc, rb_vel);
        //end
        moveHome(rb_ip);
    }

    void RepairOperations::moveHome(const std::string &rb_ip) {
        connect(rb_ip);
        robot_->moveJ({-3.08, -0.94, -1.33, -1.65, 1.55, 0.0}, rb_acc + 0.1  , rb_vel + 0.5);
    }

    void RepairOperations::grindArea(const std::string &rb_ip, const PoseArray & area){
        robot_->moveL({0.412, -0.133, 0.450, 3.121, -0.007, -0.044}, rb_acc + 0.1, rb_vel + 0.1);
        robot_->moveL({0.412, -0.133, 0.7 , 3.121, -0.007, -0.044}, rb_acc, rb_vel);

        robot_->setPayload(2.2, {-0.008,-0.02,0.062});
        robot_->setTcp({0.00344,0.13178,0.25004,0.0001,3.0982,0.4994});
        
        moveHome(rb_ip);
        if(area.poses.size() < 4) {
            std::cerr << "Need Area of operation " << std::endl;
            return;
        }
        std::vector<Vec3> corner;
        for (int i = 0; i < 4; ++i) {
            const auto &p = area.poses[i].position;
            corner.emplace_back(-p.x, -p.y, p.z);
        }
        auto path = buildPath(corner, 0.01, 0.01, 2, 0.005);
        executePath(path, 0.1, 0.1);   

    }

    void RepairOperations::getGrinder(const std::string &rb_ip) {
        moveHome(rb_ip);
        robot_->setTcp({0,0,0,0,0,0});
        robot_->moveJ({deg2rad(6.22), deg2rad(-90.27), deg2rad(-48.50),
                      deg2rad(-131.23), deg2rad(88.76), deg2rad(-83.74)}, rb_acc + 0.12, rb_vel + 0.12 );
        
        robot_->moveL({0.412, -0.133, 0.450, 3.121, -0.007, -0.044}, rb_acc + 0.1, rb_vel + 0.1);
        robot_->moveL({0.412, -0.133, 0.41834, 3.121, -0.007, -0.044}, rb_acc, rb_vel);

    }
    void RepairOperations::returnGrinder(const std::string &ip) {
        robot_->moveJ({deg2rad(6.22), deg2rad(-90.27), deg2rad(-48.50),
                    deg2rad(-131.23), deg2rad(88.76), deg2rad(-83.74)}, rb_acc + 0.12, rb_vel + 0.12 );

        robot_->moveL({0.412, -0.133, 0.450, 3.121, -0.007, -0.044}, rb_acc + 0.1, rb_vel + 0.1);
        robot_->moveL({0.412, -0.133, 0.41834, 3.121, -0.007, -0.044}, rb_acc, rb_vel);
        robot_->setTcp({0,0,0,0,0,0});
    }              

    Vec3 RepairOperations::normalFromCorners(const std::vector<Vec3> &c) {
        Vec3 n = (c[2] - c[0]).cross(c[1] - c[0]);
        if (n.z() < 0) n = -n;
        return n.normalized();
    }
    Vec3 RepairOperations::rpyFromNormal(const Vec3 &n) {
        Vec3 z(0,0,1), axis = z.cross(n);
        double angle = std::acos(z.dot(n));
        if (axis.norm() < 1e-6) axis = Vec3::UnitX();
        Eigen::Matrix3d R(Eigen::AngleAxisd(angle, axis.normalized()));
        return { std::atan2(R(2,1),R(2,2)),
                std::asin(-R(2,0)),
                std::atan2(R(1,0),R(0,0)) };
    }

    std::vector<std::vector<double>> RepairOperations::buildPath(const std::vector<Vec3> &c, double grid, double lift, int layers, double step_down) {
        /* ❶ copy python preprocessing */
        Vec3 op1=c[0], op2=c[1], op3=c[2], op4=c[3];
        Vec3 normal = normalFromCorners({op1,op2,op3});
        auto offs    = [&](const Vec3& p,double d){ return p - d*normal; };

        /* ❷ container for ALL layers */
        std::vector<std::vector<double>> wp;

        /* ❸ loop over layers (= “count” in python) */
        for (int L=0; L<layers; ++L) {
            double lower = step_down * L;               // = lowerDistance*count
            Vec3 p1 = offs(op1, lower);
            Vec3 p2 = offs(op2, lower);
            Vec3 p3 = offs(op3, lower);
            Vec3 p4 = offs(op4, lower);

            normal = normalFromCorners({p1,p2,p3});    // recalc after offset
            Vec3 move  = p2 - p1;
            Vec3 shift = (p4 - p1).normalized()*grid;
            int  passes= static_cast<int>((p4-p1).norm()/grid) + 1;

            /* ❹ actual stripes inside one layer */
            Vec3 cur = p1;
            for (int stripe=0; stripe<passes; ++stripe) {
                Vec3 end = cur + move;

                auto push = [&](const Vec3 &p) {
                    wp.push_back({p.x(), p.y(), p.z(), 0,0,0});   // rx,ry,rz→0
                };

                push(cur);
                push(end);
                push(end + lift*normal);

                if (stripe < passes-1) {
                    Vec3 nextHigh = cur + shift + lift*normal;
                    Vec3 nextLow  = nextHigh - lift*normal;
                    push(nextHigh);
                    push(nextLow);
                    cur = nextLow;
                }
            }
        }
        return wp;
    }

    void RepairOperations::executePath(const std::vector<std::vector<double>> &wp, double acc, double vel) {
        for (const auto &p : wp) 
            robot_->moveL(p, rb_acc, rb_vel);
    }

}