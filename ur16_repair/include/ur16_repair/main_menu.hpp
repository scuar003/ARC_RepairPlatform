#ifndef MAIN_MENU_HPP
#define MAIN_MENU_HPP

/* 
The Main Menu 
    is an Interactive Marker (Menu)
    is a client that request operations to a server 
    could also be a supervisor of all operations 
*/


#include <string>
#include <iostream>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>

//Alias
using Marker = visualization_msgs::msg::Marker;
using IntMarker = visualization_msgs::msg::InteractiveMarker;
using IntControl = visualization_msgs::msg::InteractiveMarkerControl;

class MainMenu {
    public:
        MainMenu();
        IntMarker createMenu(const std::string &frame_id);
    private:
        Marker makeMenu();
        IntControl makeMenuControl(const Marker &p);
        IntMarker makeMenuMarker (const std::string &frame_id, const Marker &p);
};

#endif