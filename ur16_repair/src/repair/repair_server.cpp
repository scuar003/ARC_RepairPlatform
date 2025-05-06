#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <repair_interface/action/repair_action.hpp>
#include <ur16_repair/repair/repair_operations.hpp>

using namespace std::placeholders;
using RepairCommand = repair_interface::action::RepairAction;
using RepairGoalHandle = rclcpp_action::ServerGoalHandle<RepairCommand>;
 
class RepairServer : public rclcpp::Node {
    public:
        RepairServer() : Node ("repair_server") {
            repair_server = rclcpp_action::create_server<RepairCommand>(
                this, 
                "repair_server",
                std::bind(&RepairServer::goalCb, this, _1, _2),
                std::bind(&RepairServer::cancelCb, this, _1),
                std::bind(&RepairServer::executeCb, this, _1));
                RCLCPP_INFO(this->get_logger(), "Repair Server Ready to receive commnads");
            }
    private:
        rclcpp_action::GoalResponse goalCb (const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RepairCommand::Goal> goal) {
            (void) uuid;
            RCLCPP_INFO(this->get_logger(), "Received goal ... ");
            if (goal->command == " ") {
                RCLCPP_WARN(this->get_logger(), " Rejecting goal, operation is not vailid ");
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(this->get_logger(), "Acepting Goal ");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse cancelCb (const std::shared_ptr<RepairGoalHandle> goal_handle) {
             (void) goal_handle;
             RCLCPP_INFO(this->get_logger(), "Received a cancel Request ");
             return rclcpp_action::CancelResponse::ACCEPT;
        }

        void executeCb (const std::shared_ptr<RepairGoalHandle> goal_handle) {
            std::string cmd = goal_handle -> get_goal() -> command;
            auto result = std::make_shared<RepairCommand::Result>();
            RCLCPP_INFO(this->get_logger(), "Executing '%s' operation", cmd.c_str());
            repair_op.scanEnv("192.187.6.56");
            result->completed = true;
            goal_handle->succeed(result);
        }

        rclcpp_action::Server<RepairCommand>::SharedPtr repair_server;
        ur16repair::RepairOperations repair_op;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RepairServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}