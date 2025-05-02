#ifndef REPAIR_MAIN_MENU_HPP
#define REPAIR_MAIN_MENU_HPP


#include <string>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

namespace ur16repair {
//Alias
using Marker = visualization_msgs::msg::Marker;
using IntMarker = visualization_msgs::msg::InteractiveMarker;
using IntControl = visualization_msgs::msg::InteractiveMarkerControl;
using Server = interactive_markers::InteractiveMarkerServer;
using Menu = interactive_markers::MenuHandler;
using MarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using namespace std::placeholders;

class MainMenu {
    public:
        MainMenu();
        void addToServer(Server &server, const std::string &frame_id);
    private:
        void buildEntries();
        void feedbackCb(const MarkerFeedback::ConstSharedPtr &fb);
        Marker makeMenu();
        IntControl makeMenuControl(const Marker &p);
        IntMarker makeMenuMarker (const std::string &frame_id, const Marker &p);
        Menu menu_handler;
        Menu::EntryHandle menu_repair, menu_detect_surfaces, menu_scan_env,
                          menu_home;
};

}

#endif