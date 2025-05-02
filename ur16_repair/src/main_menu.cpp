#include <ur16_repair/main_menu.hpp>

MainMenu::MainMenu() {}

IntMarker MainMenu::createMenu(const std::string &frame_id) {
    std::cout << "Creating Main Menu" << std::endl;
    auto marker = makeMenu();
    return makeMenuMarker(frame_id, marker);
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
        rotateY_control.name = "rotate_x";
        rotateY_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateY_control.orientation.w = 1;
        rotateY_control.orientation.x = 0;
        rotateY_control.orientation.y = 1;
        rotateY_control.orientation.z = 0;
        menu.controls.push_back(rotateY_control);

        IntControl rotateZ_control;
        rotateZ_control.name = "rotate_x";
        rotateZ_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateZ_control.orientation.w = 1;
        rotateZ_control.orientation.x = 1;
        rotateZ_control.orientation.y = 0;
        rotateZ_control.orientation.z = 0;
        menu.controls.push_back(rotateZ_control);

        return menu;
}

