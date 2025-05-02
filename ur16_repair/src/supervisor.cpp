#include <rclcpp/rclcpp.hpp>
#include <ur16_repair/main_menu.hpp>


class Supervisor : public rclcpp::Node {
    public:
        Supervisor () : Node ("Supervisor" ) {

        }

        void init() {
            declare_parameter("frame_id", "map");
            frame_id = get_parameter("frame_id").as_string();
            server_ = std::make_unique<ur16repair::Server>("menu_server", shared_from_this());\
            main_menu_.addToServer(*server_, frame_id);
            server_->applyChanges();
            RCLCPP_INFO(get_logger(), "Main Menu loaded ");
     
        }

    private:
            // Actions - Interactions 
        std::string frame_id;
        std::unique_ptr<ur16repair::Server> server_;
        ur16repair::MainMenu main_menu_;

};


int main (int arg, char** argv) {
    rclcpp::init(arg, argv);
    auto node = std::make_shared<Supervisor>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}