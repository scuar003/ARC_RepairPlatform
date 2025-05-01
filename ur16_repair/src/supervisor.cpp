#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include <ur16_repair/main_menu.hpp>

using Server = interactive_markers::InteractiveMarkerServer;
using Menu = interactive_markers::MenuHandler;
using MarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;

class Supervisor : public rclcpp::Node {
    public:
        Supervisor () : Node ("Supervisor" ) {

        }

        void init() {
            server_ = std::make_unique<Server>("menu_server", shared_from_this());
            startMenu();
            MainMenu mainMenu;
            server_->insert(mainMenu.createMenu(1.0));
            menu_handler_.apply(*server_, "Main_Menu");
            server_->applyChanges();
            RCLCPP_INFO(this->get_logger(), "Main Menu has been loaded");
        }

    private:
        void startMenu() {
            // Actions - Interactions 
            menu_repair = menu_handler_.insert("Repair Operations");
            menu_detect_surfaces = menu_handler_.insert("Detect Surfaces");
            menu_scan_env = menu_handler_.insert("Scan Environment"); 
        }
        
        std::unique_ptr<Server> server_;
        Menu menu_handler_;
        Menu::EntryHandle menu_repair, menu_detect_surfaces, menu_scan_env,
                          menu_home, menu_3d_mouse;
        
};