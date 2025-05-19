#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> 
#include <repair_interface/action/repair_action.hpp>
#include <ur16_repair/repair/repair_operations.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::placeholders;
using namespace std::chrono;
using RepairCommand = repair_interface::action::RepairAction;
using RepairGoalHandle = rclcpp_action::ServerGoalHandle<RepairCommand>;
using ToolClientHandle = rclcpp_action::ClientGoalHandle<RepairCommand>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseArray = geometry_msgs::msg::PoseArray;
using SetBool = std_srvs::srv::SetBool;
using String = std_msgs::msg::String;

 
class RepairServer : public rclcpp::Node {
    public:
        RepairServer() : Node ("repair_server") {
            cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            this->declare_parameter("robot_ip", "192.168.1.85");
            this->declare_parameter("target_frame", "base_link");
            robot_ip = this->get_parameter("robot_ip").as_string();
            target_frame = this->get_parameter("target_frame").as_string();

            cloud_sub = this->create_subscription<PointCloud2>("/camera/depth/color/points", 10, std::bind(&RepairServer::cloudCb, this, _1));
            maping_client = this->create_client<SetBool>("set_publish_active");
            tool_client = rclcpp_action::create_client<RepairCommand>(this, "tool_server");
            poses_pub_ = this->create_publisher<repairs::PoseArray>("rendered_surfaces", 10);

            tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
            auto tool_cb = std::bind(&RepairServer::toolState, this, _1);
            repair_op = repairs::RepairOperations(tf_buffer, target_frame, tool_cb);

            repair_server = rclcpp_action::create_server<RepairCommand>(
                this, 
                "repair_server",
                std::bind(&RepairServer::goalCb, this, _1, _2),
                std::bind(&RepairServer::cancelCb, this, _1),
                std::bind(&RepairServer::executeCb, this, _1),
                rcl_action_server_get_default_options(), cb_group );
                RCLCPP_INFO(this->get_logger(), "Repair Server Ready to receive commands");

            
            }
    private:

        //--------------------- Actiion service ---------------------
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
            std::vector<repairs::CusEigen> area = goal_handle -> get_goal() -> area;
            auto result = std::make_shared<RepairCommand::Result>();
            RCLCPP_INFO(this->get_logger(), "Executing '%s' operation", cmd.c_str());
            //start execution of cmd 
            if(cmd == "detect_surfaces" ) {
                auto corners = PoseArray();
                corners = repair_op.detect(cloud_msg);
                poses_pub_->publish(corners);
            }
            if(cmd == "scan_env" ) {
                startMapping();
                repair_op.scanEnv(robot_ip);
                stopMapping();
            }
            if (cmd == "home") {
                repair_op.moveHome(robot_ip);
                RCLCPP_INFO(this->get_logger(), "Robot at Home Position ");
            }
            if(cmd == "grind") {
                //repair_op.getGrinder(robot_ip);
                repair_op.grindArea(robot_ip, area);
                //repair_op.returnGrinder(robot_ip);
            }
            //if (cmd == "tool_unlock") toolState(cmd);
            //if (cmd == "tool_lock") toolState(cmd);
       
             
            result->completed = true;
            goal_handle->succeed(result);
        }

        //-----------------------Helpers
        //pointcloud callback 
        void cloudCb(const PointCloud2::SharedPtr msg) {cloud_msg = msg;}

        void startMapping() {
            while(!maping_client -> wait_for_service(1s)) {
                RCLCPP_INFO(this->get_logger(), "Wating for service...");
            }
            auto request = std::make_shared<SetBool::Request>();
            request ->data = true;
            maping_client -> async_send_request(request);
        }
        void stopMapping() {
            while(!maping_client -> wait_for_service(1s)) {
                RCLCPP_INFO(this->get_logger(), "Wating for service...");
            }
            auto request = std::make_shared<SetBool::Request>();
            request ->data = false;
            maping_client -> async_send_request(request);
        }
        bool toolState(const std::string &cmd) {
            using namespace std::chrono_literals;
            /* 1 — make sure the action‑server exists */
            if (!tool_client->wait_for_action_server(2s)) {
                RCLCPP_ERROR(get_logger(), "tool_server not up");
                return false;
            }
            /* 2 — send the goal */
            RepairCommand::Goal goal_msg;
            goal_msg.command = cmd;
            auto goal_future = tool_client->async_send_goal(goal_msg);

            /* With a MultiThreadedExecutor the result‑handling thread(s) are
            * already running, so we can just block on the future here.          */
            if (goal_future.wait_for(5s) != std::future_status::ready) {
                RCLCPP_ERROR(get_logger(), "Timed‑out sending goal '%s'", cmd.c_str());
                return false;
            }
            auto goal_handle = goal_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Goal '%s' rejected by tool_server", cmd.c_str());
                return false;
            }
            /* 3 — wait for the action result */
            auto result_future = tool_client->async_get_result(goal_handle);
            if (result_future.wait_for(30s) != std::future_status::ready) {
                RCLCPP_ERROR(get_logger(), "Timed‑out waiting for result of '%s'", cmd.c_str());
                return false;
            }
            auto result_code = result_future.get().code;
            return result_code == rclcpp_action::ResultCode::SUCCEEDED;
        }

        //members - atributes
        rclcpp_action::Server<RepairCommand>::SharedPtr repair_server;
        rclcpp_action::Client<RepairCommand>::SharedPtr tool_client;
        rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub;
        rclcpp::CallbackGroup::SharedPtr cb_group;
        rclcpp::Client<SetBool>::SharedPtr maping_client;
        rclcpp::Publisher<repairs::PoseArray>::SharedPtr poses_pub_;

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