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
        robot_->moveJ({-1.57, -0.94, -1.33, -1.65, 1.55, 0.0}, rb_acc, rb_vel);
    }

    void RepairOperations::grindArea(const std::string& rb_ip, const PoseArray & area){

    }

}