#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <repair_interface/action/repair_action.hpp>
#include <ur16_repair/ui/main_menu.hpp>


using RepairCommand = repair_interface::action::RepairAction;
using RepairGoalHandle = rclcpp_action::ClientGoalHandle<RepairCommand>;
using namespace std::placeholders;

class Supervisor : public rclcpp::Node {
    public:
        Supervisor () : Node ("Supervisor" ) {
            declare_parameter("frame_id", "map");
        }
        void init() {
            frame_id = get_parameter("frame_id").as_string();

            menu_server_ = std::make_unique<ur16repair::Server>(
                    "menu_server",
                     shared_from_this());
            
            repair_client = rclcpp_action::create_client<RepairCommand>(
                            shared_from_this(),
                            "repair_server");
            
            main_menu_ = std::make_unique<ur16repair::MainMenu>(
                         std::bind(&Supervisor::onMenuSelection, this, _1));
            
            main_menu_->addToServer(*menu_server_, frame_id);
            menu_server_->applyChanges();
            RCLCPP_INFO(get_logger(), "Main Menu is ready in FRAME: '%s'", frame_id.c_str());
        }
        void cancelGoal() {
            RCLCPP_INFO(get_logger(), "Request to cancel execution");
            repair_client->async_cancel_all_goals();
        }

    private:
        void onMenuSelection(const std::string& cmd) {
            std::cout << cmd << std::endl;
            if (!repair_client->wait_for_action_server()) {
                RCLCPP_ERROR(get_logger(), "Repair Action Server Not available ");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(get_logger(), "Menu selected: %s -> sending goal ", cmd.c_str());
            
            auto goal = RepairCommand::Goal();
            goal.command = cmd;
            auto options = rclcpp_action::Client<RepairCommand>::SendGoalOptions();
            options.goal_response_callback = std::bind(&Supervisor::responseCb, this, _1);
            options.result_callback = std::bind(&Supervisor::resultCb, this, _1);
            options.feedback_callback = std::bind(&Supervisor::feedbackCb, this, _1, _2);
            repair_client->async_send_goal(goal, options);

        }
        void responseCb(const RepairGoalHandle::SharedPtr &goal_handle) {
            if(!goal_handle)
                RCLCPP_INFO(get_logger(), "Goal Rejected ");
            else
                RCLCPP_INFO(get_logger(), "Goal Accepted ");
        }
        void resultCb(const RepairGoalHandle::WrappedResult &result) {
            auto status = result.code;
            if (status == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(get_logger(), "Goal Succeeded ");
            else if (status == rclcpp_action::ResultCode::CANCELED)
                RCLCPP_INFO(get_logger(), "Goal Canceled ");
            else if (status == rclcpp_action::ResultCode::ABORTED)
                RCLCPP_INFO(get_logger(), "Goal Aborted ");
        
        }
        void feedbackCb(const RepairGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const RepairCommand::Feedback> fb) {

            (void) goal_handle;
            std::string c_status = fb->status;
            RCLCPP_INFO(get_logger(), "Feedback: %s", c_status.c_str());
            if (c_status == "cancel_action")
                cancelGoal();
                            
        }
            // Actions - Interactions 
        std::string frame_id;
        std::unique_ptr<ur16repair::Server> menu_server_;
        std::unique_ptr<ur16repair::MainMenu> main_menu_;
        rclcpp_action::Client<RepairCommand>::SharedPtr repair_client; 
        

};


int main (int arg, char** argv) {
    rclcpp::init(arg, argv);
    auto node = std::make_shared<Supervisor>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}