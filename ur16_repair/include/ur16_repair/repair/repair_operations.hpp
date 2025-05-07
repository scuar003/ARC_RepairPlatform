#ifndef REPAIR_OPERATIONS_HPP
#define REPAIR_OPERATIONS_HPP

#include <string>
#include <ur_rtde/rtde_control_interface.h>

namespace repairs {

using namespace ur_rtde;

class RepairOperations {
    public:
        RepairOperations();
        void scanEnv(const std::string&rb_ip);
};

}


#endif