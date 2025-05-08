#include <ur16_repair/repair/repair_operations.hpp>
#include <iostream>

namespace repairs {

    RepairOperations::RepairOperations() {}

    void RepairOperations::scanEnv(const std::string &rb_ip){
        std::cout << "Connecting to robot at: " << rb_ip << std::endl;
        // RTDEControlInterface robot_(rb_ip);
        // auto acc = 0.08, vel = 0.08;
        // //Start
        // robot_.moveJ({-1.57, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
        // //right
        // robot_.moveJ({-3.25, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
        // //left
        // robot_.moveJ({0.83, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
        // //end
        // robot_.moveJ({-1.57, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
    }

}