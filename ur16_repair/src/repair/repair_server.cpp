#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> 
#include <repair_interface/action/repair_action.hpp>
#include <ur16_repair/repair/repair_operations.hpp>

using namespace std::placeholders;
using RepairCommand = repair_interface::action::RepairAction;
using RepairGoalHandle = rclcpp_action::ServerGoalHandle<RepairCommand>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseArray = geometry_msgs::msg::PoseArray;

 
class RepairServer : public rclcpp::Node {
    public:
        RepairServer() : Node ("repair_server") {
            cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->declare_parameter("robot_ip", "192.168.1.85");
            this->declare_parameter("target_frame", "base_link");
            robot_ip = this->get_parameter("robot_ip").as_string();
            target_frame = this->get_parameter("target_frame").as_string();

            tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
            repair_op = repairs::RepairOperations(tf_buffer, target_frame);

            repair_server = rclcpp_action::create_server<RepairCommand>(
                this, 
                "repair_server",
                std::bind(&RepairServer::goalCb, this, _1, _2),
                std::bind(&RepairServer::cancelCb, this, _1),
                std::bind(&RepairServer::executeCb, this, _1),
                rcl_action_server_get_default_options(), cb_group );
                RCLCPP_INFO(this->get_logger(), "Repair Server Ready to receive commands");

            cloud_sub = this->create_subscription<PointCloud2>("/camera/depth/color/points", 10, std::bind(&RepairServer::cloudCb, this, _1));
            }
    private:
        //pointcloud callback 
        void cloudCb(const PointCloud2::SharedPtr msg) {cloud_msg = msg;}

        // Actiion service 
        rclcpp_action::GoalResponse goalCb (const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RepairCommand::Goal> goal) {
            (void) uuid;
            RCLCPP_INFO(this->get_logger(), "Received goal ... ");
            if (goal->command.empty()) {
                RCLCPP_WARN(this->get_logger(), " Rejecting goal, operation is not valid ");
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(this->get_logger(), "Accepting Goal ");
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
            //start execution of cmd 
            if(cmd == "detect_surfaces" ) {
                auto corners = PoseArray();
                corners = repair_op.detect(cloud_msg);

                // repair_op.displayPlanes(corners); // not yet implemented    
            }
            if(cmd == "scan_env" ) 
                repair_op.scanEnv(robot_ip);
             
            
            result->completed = true;
            goal_handle->succeed(result);
        }


        //members - atributes
        rclcpp_action::Server<RepairCommand>::SharedPtr repair_server;
        rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub;
        rclcpp::CallbackGroup::SharedPtr cb_group;
        PointCloud2::SharedPtr cloud_msg;
        repairs::RepairOperations repair_op;
        std::string robot_ip, target_frame;

        //transforms buffers
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RepairServer>();
    rclcpp::executors::MultiThreadedExecutor exec; 
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}