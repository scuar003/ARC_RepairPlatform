#include <ur16_repair/ui/main_menu.hpp>
#include <iostream>

namespace ur16repair {
    MainMenu::MainMenu(SelectionCb cb) : selection_cb(std::move(cb)) {}

    void MainMenu::addToServer(Server &server, const std::string &frame_id) {
        buildEntries();
        auto marker = makeMenuMarker(frame_id, makeMenu());
        server.insert(marker);
        menu_handler.apply(server, marker.name);
    }

    void MainMenu::buildEntries() {
        auto on_event = [this](const MarkerFeedback::ConstSharedPtr &fb) {feedbackCb(fb);};
        //Repair Operations
        menu_repair = menu_handler.insert("Repair Operations");
        menu_detect_surfaces = menu_handler.insert(menu_repair, "Detect Surfaces", on_event);
        menu_scan_env = menu_handler.insert(menu_repair, "Scan Environment", on_event);
        //Move 
        menu_home = menu_handler.insert("Home", on_event);
        
    }

    void MainMenu::feedbackCb(const MarkerFeedback::ConstSharedPtr &fb) {
        if (fb -> event_type != MarkerFeedback::MENU_SELECT) return;

        std::string cmd;
        if (fb -> menu_entry_id == menu_detect_surfaces) cmd = "detect_surfaces";
        else if (fb -> menu_entry_id == menu_scan_env) cmd = "scan_env";
        else if (fb -> menu_entry_id == menu_home) cmd = "home";

        if (!cmd.empty() && selection_cb) selection_cb(cmd);
        
    }

    Marker MainMenu::makeMenu() {
        Marker menuType;
        menuType.type = Marker::SPHERE;
        // type, scale, color
        menuType.scale.x = 1.0 * 0.25;
        menuType.scale.y = 1.0 * 0.25;
        menuType.scale.z = 1.0 * 0.25;
        menuType.color.r = 1.0f;
        menuType.color.g = 1.0f;
        menuType.color.b = 1.0f;
        menuType.color.a = 1.0;

        return menuType;
    }

    IntControl MainMenu::makeMenuControl(const Marker &p) {
        IntControl menuControl;
        menuControl.always_visible = true;
        menuControl.markers.push_back(p);
        menuControl.interaction_mode = IntControl::BUTTON;

        return menuControl;
    }

    IntMarker MainMenu::makeMenuMarker(const std::string &frame_id, const Marker &p) {
        IntMarker menu;
            menu.header.frame_id = frame_id;
            menu.pose.position.x = 1.0;
            menu.pose.position.y = 1.0;
            menu.pose.position.z = 1.0;
            menu.name = "Main_Menu";
            menu.description = "Menu";
            menu.scale = 0.25;

            menu.controls.push_back(makeMenuControl(p));

            // Arrow movement

            IntControl control;
            control.orientation.w = 1;
            // move x
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "move_x";
            control.interaction_mode = IntControl::MOVE_AXIS;
            menu.controls.push_back(control);
            //move y
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "move_y";
            control.interaction_mode = IntControl::MOVE_AXIS;
            menu.controls.push_back(control);
            // move y
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "move_z";
            control.interaction_mode = IntControl::MOVE_AXIS;
            menu.controls.push_back(control);

            // rotation controls
            IntControl rotateX_control;
            rotateX_control.name = "rotate_x";
            rotateX_control.interaction_mode = IntControl::ROTATE_AXIS;
            rotateX_control.orientation.w = 1;
            rotateX_control.orientation.x = 1;
            rotateX_control.orientation.y = 0;
            rotateX_control.orientation.z = 0;
            menu.controls.push_back(rotateX_control);

            IntControl rotateY_control;
            rotateY_control.name = "rotate_y";
            rotateY_control.interaction_mode = IntControl::ROTATE_AXIS;
            rotateY_control.orientation.w = 1;
            rotateY_control.orientation.x = 0;
            rotateY_control.orientation.y = 1;
            rotateY_control.orientation.z = 0;
            menu.controls.push_back(rotateY_control);

            IntControl rotateZ_control;
            rotateZ_control.name = "rotate_z";
            rotateZ_control.interaction_mode = IntControl::ROTATE_AXIS;
            rotateZ_control.orientation.w = 1;
            rotateZ_control.orientation.x = 1;
            rotateZ_control.orientation.y = 0;
            rotateZ_control.orientation.z = 0;
            menu.controls.push_back(rotateZ_control);

            return menu;
    }

}