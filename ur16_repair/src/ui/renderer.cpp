#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include <ur16_repair/repair/plane_geometry.hpp>
#include <repair_interface/action/repair_action.hpp>
#include <repair_interface/msg/eigen_msg.hpp>


#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

using namespace std::placeholders;
using PoseArray = geometry_msgs::msg::PoseArray;
using Marker = visualization_msgs::msg::Marker;
using Server = interactive_markers::InteractiveMarkerServer;
using IntMarker = visualization_msgs::msg::InteractiveMarker;
using IntControl = visualization_msgs::msg::InteractiveMarkerControl;
using Menu = interactive_markers::MenuHandler;
using MarkerFb = visualization_msgs::msg::InteractiveMarkerFeedback;
using RepairCommand = repair_interface::action::RepairAction;
using RepairGoalHandle = rclcpp_action::ClientGoalHandle<RepairCommand>;
using CusEigen = repair_interface::msg::EigenMsg; // custom eigen msg;

class Renderer : public rclcpp::Node {
    public:
        Renderer() : Node ("renderer") {
            declare_parameter("frame_id", "map");
        }
        void init () {
            frame_id = get_parameter("frame_id").as_string();

            renderer_sub_= create_subscription<PoseArray>("rendered_surfaces", rclcpp::SensorDataQoS(), std::bind(&Renderer::showSurfacesCb, this, _1));
            renderer_pub_= create_publisher<Marker>("surfaces", 10);
            selected_area_pub_ = create_publisher<Marker>("selected_area", 10);
            selected_area_sub_ = create_subscription<plane_geometry::PointStamped>("/clicked_point", 10, std::bind(&Renderer::areaCb, this, _1));
            repair_client = rclcpp_action::create_client<RepairCommand>(shared_from_this(),"repair_server");
            int_server_= std::make_unique<Server>("selector", shared_from_this());
            startMenu();

        }
        void cancelGoal() {
            RCLCPP_INFO(get_logger(), "Request to cancel execution");
            repair_client->async_cancel_all_goals();
        }
    private:
        void showSurfacesCb(const PoseArray::SharedPtr poses) {
            auto v = plane_geometry::poses2vectors(*poses);
            int n = v.size();
            if (!n % 4) {
                RCLCPP_ERROR(get_logger(), "Ops, %d not enough plane corners!" , n);
                return;
            }
            planes_.clear();
            for (int i = 0; i < n; i+= 4) 
                planes_.push_back(plane_geometry::Plane(v[i], v[i + 1], v[i + 2], v[i + 3]));
            RCLCPP_INFO(get_logger(), "%ld planes detected", planes_.size());
            updateRenderer();
        }

// ------------------- Helpers --------------------------
        void updateRenderer() {
            clearPlanes();
            for (long unsigned int i = 0; i < planes_.size(); i++)
                displayPlanes(planes_[i],  i);
        }
        void clearPlanes() {
            Marker m; 
            m.header.stamp = rclcpp::Clock().now(); // do I need this ?
            m.action = Marker::DELETEALL;
            m.header.frame_id = frame_id;
            renderer_pub_->publish(m);
            RCLCPP_INFO(get_logger(), "Planes have been deleted ready to display new planes");

        }
        

        void displayPlanes(const plane_geometry::Plane &plane, int id) {
            addPlane(plane, id);
            addEdge(plane, id);
            addSelector(plane, id);
            
        }
        void addPlane(const plane_geometry::Plane &plane, int id) {
            Marker p; 
            p.type = Marker::TRIANGLE_LIST;
            p.header.frame_id = frame_id; 
            p.header.stamp = rclcpp::Clock().now();
            p.action = Marker::ADD;
            p.ns = "surfaces"; 
            p.id = id;
            p.pose.orientation.w = 1.0f;
            p.scale.x = 1.0f; 
            p.scale.y = 1.0f; 
            p.scale.z = 1.0f;
            // Triangle 1
            p.points.push_back(plane_geometry::vector2point(plane.vertex(0)));
            p.points.push_back(plane_geometry::vector2point(plane.vertex(1)));
            p.points.push_back(plane_geometry::vector2point(plane.vertex(2)));
            // Triangle 2
            p.points.push_back(plane_geometry::vector2point(plane.vertex(1)));
            p.points.push_back(plane_geometry::vector2point(plane.vertex(3)));
            p.points.push_back(plane_geometry::vector2point(plane.vertex(2)));
            // Color
            p.color.r = 1.0f;
            p.color.g = 0.0f;
            p.color.b = 0.0f;
            p.color.a = 0.25f;
            renderer_pub_->publish(p); 
        }
        void addEdge(const plane_geometry::Plane &plane, int id) {
            Marker marker;
            marker.type = Marker::LINE_STRIP;
            marker.header.frame_id = "base_link";
            marker.header.stamp = rclcpp::Clock().now();
            marker.action = Marker::ADD;
            marker.ns = "repair_surfaces";
            marker.id = id + 200;
            marker.pose.orientation.w = 1.0f;
            marker.scale.x = 0.01f;
            // Vectors
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(0)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(1)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(3)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(2)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(0)));
            // Color
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
            renderer_pub_->publish(marker);
        }
        void addSelector(const plane_geometry::Plane &plane, int id){
            // Create an interactive marker
            IntMarker int_marker;
            int_marker.header.frame_id = frame_id;
            int_marker.header.stamp = rclcpp::Clock().now();
            int_marker.name = "repair_surface_" + std::to_string(id);
            int_marker.description = "Surface " + std::to_string(id);

            // Set the position of the interactive marker
            auto centroid = plane.centroid();
            int_marker.pose.position.x = centroid[0];
            int_marker.pose.position.y = centroid[1];
            int_marker.pose.position.z = centroid[2];
            int_marker.scale = 0.1; // Adjust the scale of the interactive marker if necessary

            // Create a sphere control for the interactive marker
            IntControl button_control;
            button_control.interaction_mode = IntControl::BUTTON;
            button_control.name = "button_control";

            // Add a sphere to the button control
            Marker marker;
            marker.type = Marker::SPHERE;
            marker.scale.x = 0.05f; // Sphere size
            marker.scale.y = 0.05f;
            marker.scale.z = 0.05f;
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;

            button_control.markers.push_back(marker);
            button_control.always_visible = true; // Make the control always visible

            // Add the control to the interactive marker
            int_marker.controls.push_back(button_control);
            menu_handler_.apply(*int_server_, "TaskAction");

            // Add the interactive marker to the server
            int_server_->insert(int_marker, std::bind(&Renderer::selectorFb, this, _1));
            // Apply changes to the interactive marker server
            int_server_->applyChanges();
        }
        void selectorFb (const MarkerFb::ConstSharedPtr &fb) {
            if (fb->event_type == MarkerFb::BUTTON_CLICK) {
                std::cout << "Button for surface " << fb->marker_name << " pressed." << std::endl;
                std::string selected_id_str = fb->marker_name.substr(std::string("repair_surface_").length());
                int selected_id = std::stoi(selected_id_str);

                // Clear all interactive markers except the selected one
                for (size_t i = 0; i < planes_.size(); ++i) {
                    std::string marker_name = "repair_surface_" + std::to_string(i);
                    if (i != selected_id) {
                        int_server_->erase(marker_name);
                    }
                }
                int_server_->clear();          // ⚠ deletes every interactive marker
                // Clear all visualization markers except those related to the selected sphere/plane
                createSelectedPlane(planes_[selected_id], selected_id);
            }
        }
        void createSelectedPlane(const plane_geometry::Plane &plane, int id) {
            clearPlanes();
            addCorners(plane, id);
            auto p = makePlane(plane, id);
            auto int_p = makeMenuPlane("TaskAction", plane);
            int_server_->insert(int_p);
            menu_handler_.apply(*int_server_, "TaskAction");
            int_server_->applyChanges();
        }
        void addCorners(const plane_geometry::Plane &plane, int id) {
            for (int i = 0; i < 4; ++i) {  // Assuming each plane has 4 corners
                IntMarker int_marker;
                int_marker.header.frame_id = frame_id;
                int_marker.header.stamp = rclcpp::Clock().now();
                int_marker.name = "corner_" + std::to_string(id) + "_" + std::to_string(i);
                int_marker.description = "Corner " + std::to_string(i);
    
                int_marker.pose.position = plane_geometry::vector2point(plane.vertex(i));
                int_marker.scale = 0.05; // Adjust the scale of the interactive marker if necessary
    
                IntControl move_control;
                move_control.interaction_mode = IntControl::MOVE_PLANE;
                move_control.orientation.w = 1;
                move_control.orientation.x = 0;
                move_control.orientation.y = 1;
                move_control.orientation.z = 0;
                
                move_control.always_visible = true;
    
                Marker marker;
                marker.type = Marker::SPHERE;
                marker.scale.x = 0.025f; // Sphere size
                marker.scale.y = 0.025f;
                marker.scale.z = 0.05f;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
    
                move_control.markers.push_back(marker);
                int_marker.controls.push_back(move_control);
    
                int_server_->insert(int_marker, std::bind(&Renderer::cornerFb, this, _1));
                int_server_->applyChanges();
            }
        }
        Marker makePlane(const plane_geometry::Plane &plane, int id) {
            Marker marker;
            marker.type = Marker::TRIANGLE_LIST;
            marker.header.frame_id = frame_id;
            marker.header.stamp = rclcpp::Clock().now();
            marker.action = Marker::ADD;
            marker.ns = "repair_surfaces";
            marker.id = id;
            marker.pose.orientation.w = 1.0f;
            // Scale
            marker.scale.x = 1.0f;
            marker.scale.y = 1.0f;
            marker.scale.z = 1.0f;
            // Triangle 1
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(0)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(1)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(2)));
            // Triangle 2
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(1)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(3)));
            marker.points.push_back(plane_geometry::vector2point(plane.vertex(2)));
            // Color
            marker.color.r = 1.0f;
            marker.color.g = 0.4f;
            marker.color.b = 0.0f;
            marker.color.a = 0.7f;
            return marker;
        }     
        IntControl makePlaneControl(const Marker &p_)
        {
            IntControl planeControl;
            planeControl.always_visible = true;
            planeControl.markers.push_back(p_);
            planeControl.interaction_mode = IntControl::BUTTON;
    
            return planeControl;
        }      
        IntMarker makeMenuPlane(const std::string &name, const plane_geometry::Plane &plane) {
            IntMarker int_marker;
            int_marker.header.frame_id = frame_id;
            int_marker.name = name;
            int_marker.description = "Selected Plane";
    
            // Set the position of the interactive marker to the centroid of the plane
            auto centroid = plane.centroid();
            int_marker.pose.position.x = centroid.x();
            int_marker.pose.position.y = centroid.y();
            int_marker.pose.position.z = centroid.z();
    
            // Set the orientation to match the plane's normal
         
            int_marker.pose.orientation.w = 1.0;
            int_marker.pose.orientation.x = 0.0;
            int_marker.pose.orientation.y = 1.0;
            int_marker.pose.orientation.z = 0.0;
    
            int_marker.scale = 0.4;
    
            // Add the plane visualization marker
            Marker marker = makePlane(plane, 0);
            IntControl plane_control = makePlaneControl(marker);
            int_marker.controls.push_back(plane_control);
 
            return int_marker;
        }
        void cornerFb(const MarkerFb::ConstSharedPtr &fb) {
            if (fb->event_type == MarkerFb::MOUSE_UP) {
                std::string marker_name = fb->marker_name;
                size_t first_underscore = marker_name.find('_');
                size_t last_underscore = marker_name.rfind('_');
                int plane_id = std::stoi(marker_name.substr(first_underscore + 1, last_underscore - first_underscore - 1));
                int corner_id = std::stoi(marker_name.substr(last_underscore + 1));
    
                plane_geometry::Vector new_position(fb->pose.position.x, fb->pose.position.y, fb->pose.position.z);
                planes_[plane_id].updateCorner(corner_id, new_position);
    
                // Update only the selected plane
                updateCornersPlane(plane_id);
            }
        }
        void updateCornersPlane(int id) {
            clearPlanes();
            displayPlanes(planes_[id], id);
        }
        
        void startMenu()
        {
            // first entry
            interactions = menu_handler_.insert("Interactions");
            // repair operatiopns 
            grind = menu_handler_.insert(interactions, "grind", std::bind(&Renderer::stateCb, this, _1));
            menu_handler_.setCheckState(grind, Menu::UNCHECKED);
    
            expo_marker = menu_handler_.insert(interactions, "expo_marker", std::bind(&Renderer::stateCb, this, _1));
            menu_handler_.setCheckState(expo_marker, Menu::UNCHECKED);
    
            vacum = menu_handler_.insert(interactions, "Vacum", std::bind(&Renderer::stateCb, this, _1));
            menu_handler_.setCheckState(vacum, Menu::UNCHECKED);
    
            //second entry
            clear_selection = menu_handler_.insert("Clear Selection", std::bind(&Renderer::clearSelection, this, _1));

        }
        void stateCb(const MarkerFb::ConstSharedPtr &fb) {
            handle = fb -> menu_entry_id;
            if(handle == grind) {
                menu_handler_.setCheckState(grind, Menu::CHECKED);
                menu_handler_.setCheckState(expo_marker, Menu::UNCHECKED);
                menu_handler_.setCheckState(vacum, Menu::UNCHECKED);
                repair_area.clear();
            }
            if (handle == expo_marker) {
                menu_handler_.setCheckState(expo_marker, Menu::CHECKED);
                menu_handler_.setCheckState(grind, Menu::UNCHECKED);
                menu_handler_.setCheckState(vacum, Menu::UNCHECKED);
                repair_area.clear();                
            }
            if (handle == vacum) {
                menu_handler_.setCheckState(vacum, Menu::CHECKED);
                menu_handler_.setCheckState(expo_marker, Menu::UNCHECKED);
                menu_handler_.setCheckState(grind, Menu::UNCHECKED); 
                repair_area.clear();
               
            }
    
      
            menu_handler_.reApply(*int_server_);
            int_server_ -> applyChanges();
        }
        void clearSelection (const MarkerFb::ConstSharedPtr &feedback) {
            (void) feedback;
            Marker clear_marker;
            clear_marker.header.frame_id = frame_id; // Same frame_id as the original marker
            clear_marker.header.stamp = rclcpp::Clock().now();
            clear_marker.ns = "repair_area"; // Same namespace as the original marker
            clear_marker.id = 0; // Same ID as the original marker
            clear_marker.action = Marker::DELETE; // Action set to DELETE
    
            // Publish the clear_marker to remove the previously published marker
            selected_area_pub_->publish(clear_marker);
            int_server_->erase("Execute_Repair");
            int_server_->applyChanges();
    
            RCLCPP_INFO(this->get_logger(), "Repair area marker cleared.");
    
            repair_area.clear();
        }

        void areaCb(const plane_geometry::PointStamped::ConstSharedPtr &msg) {
            // if all menu options are unckecked then ignore the point received
            // if an option is checked save 4 points
            //this can be done with a switch case 
            //called repairArea(passed the string of the checked menu option) creates linestrip of the area 
            std::string cmd;
            Menu::CheckState state;                // local temp

            menu_handler_.getCheckState(grind, state);
            if (state == Menu::CHECKED) cmd = "grind";

            menu_handler_.getCheckState(vacum, state);
            if (state == Menu::CHECKED) cmd = "vacum";

            menu_handler_.getCheckState(expo_marker, state);
            if (state == Menu::CHECKED) cmd = "expo_marker";

            if (cmd.empty()) {                     // nothing selected
                RCLCPP_WARN(get_logger(), "No operation selected – ignored point");
                return;
            }

            repair_area.push_back(msg->point);
            RCLCPP_INFO(get_logger(), "Collected %zu / 4 points for '%s'",
                        repair_area.size(), cmd.c_str());

            if (repair_area.size() == 4) {
                repairArea(cmd);                   // cmd is now the *current* one
                repair_area.clear();
            }
        }

        void repairArea(const std::string &cmd) {

            if (repair_area.size() != 4) {
                RCLCPP_ERROR(get_logger(), "Need 4 points for area to be created");
                return;
            }
            int_server_->erase("Execute_Repair");
            // Create a LINE_STRIP marker to represent the repair area
            Marker line_strip;
            line_strip.header.frame_id = frame_id;
            line_strip.header.stamp = rclcpp::Clock().now();
            line_strip.ns = "repair_area";
            line_strip.id = 0; // Unique ID for this marker
            line_strip.type = Marker::LINE_STRIP;
            line_strip.action = Marker::ADD;

            // LINE_STRIP markers use only the x component of scale, for the line width
            line_strip.scale.x = 0.005; // Specify a suitable line width

            // Set the color of the line strip
            line_strip.color.r = 1.0f;
            line_strip.color.g = 0.0f;
            line_strip.color.b = 0.0f;
            line_strip.color.a = 1.0f; // Don't forget to set the alpha!

            // Assign the points from plane_points to the marker
            for (const auto &point : repair_area)
            {
                line_strip.points.push_back(point);
            }
            // Connect the last point to the first to close the loop
            line_strip.points.push_back(repair_area[0]);

            repair_area_corners = repair_area; 

            // Publish the marker
            selected_area_pub_->publish(line_strip);

            RCLCPP_INFO(this->get_logger(), "Published repair area marker.");

            // Define an interactive marker
            IntMarker execute_marker;
            execute_marker.header.frame_id = frame_id;
            execute_marker.header.stamp = rclcpp::Clock().now();
            execute_marker.name = "Execute_Repair";
            execute_marker.description = "Press to execute repair";

            // Set the position (adjust according to your needs)
            execute_marker.pose.position.x = 0.25; // Example positions
            execute_marker.pose.position.y = 0.25;
            execute_marker.pose.position.z = 0.25;
            execute_marker.scale = 0.1; // Scale of the interactive marker

            // Create a control that will act as a button
            IntControl button_control;
            button_control.interaction_mode = IntControl::BUTTON;
            button_control.name = "button";

            // Create a marker for the button
            Marker marker;
            marker.type = Marker::CUBE;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // Add the marker to the button control
            button_control.markers.push_back(marker);
            button_control.always_visible = true;

            // Add the control to the interactive marker
            execute_marker.controls.push_back(button_control);

            // Insert the interactive marker into the server and apply changes
            std::cout << "The executer button has been Published" << std::endl;
            int_server_->insert(execute_marker, [this, cmd](const MarkerFb::ConstSharedPtr &fb) { waitToExecute(fb, cmd);});
            int_server_->applyChanges();
        }

        void waitToExecute(const MarkerFb::ConstSharedPtr &fb, const std::string &cmd){
            if (fb -> event_type != MarkerFb::BUTTON_CLICK) return;

            std::vector<CusEigen> corners;
            corners.reserve(repair_area_corners.size());
            for (const auto &pt: repair_area_corners) {
                CusEigen v;
                v.x = pt.x;
                v.y = pt.y;
                v.z = pt.z;
                corners.push_back(v);
            }
            // request the server 
            //goal -> cmd = cmd 
            // goal -> area = corner 
            //send goal
            auto goal = RepairCommand::Goal();
            goal.command = cmd;
            goal.area = corners;
            auto op = rclcpp_action::Client<RepairCommand>::SendGoalOptions();
            op.result_callback = std::bind(&Renderer::resultCb, this, _1);
            op.feedback_callback = std::bind(&Renderer::feedbackCb, this, _1,_2);
            repair_client->async_send_goal(goal, op);
            
        }
        void resultCb (const RepairGoalHandle::WrappedResult &result) {
            auto st = result.code;
            if (st == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(get_logger(), "Goal  Succeeded ");
            else if (st == rclcpp_action::ResultCode::CANCELED)
                RCLCPP_INFO(get_logger(), "Goal Canceled ");
            else if (st == rclcpp_action::ResultCode::ABORTED)
                RCLCPP_INFO(get_logger(), "Goal Aborted ");
        }
        void feedbackCb(const RepairGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const RepairCommand::Feedback> fb) {

            (void) goal_handle;
            std::string c_status = fb->status;
            RCLCPP_INFO(get_logger(), "Feedback: %s", c_status.c_str());
            if (c_status == "cancel_action")
                cancelGoal();
                            
        }
        
        rclcpp::Subscription<PoseArray>::SharedPtr renderer_sub_;
        rclcpp::Subscription<plane_geometry::PointStamped>::SharedPtr selected_area_sub_;
        rclcpp::Publisher<Marker>::SharedPtr renderer_pub_, selected_area_pub_;
        rclcpp_action::Client<RepairCommand>::SharedPtr repair_client; 
        std::unique_ptr<Server> int_server_;

        std::vector<plane_geometry::Plane> planes_;
        std::vector<plane_geometry::Point> repair_area;
        std::vector<plane_geometry::Point> repair_area_corners;

        std::string frame_id;

        //Menu 
        Menu menu_handler_;
        Menu::EntryHandle interactions, grind, expo_marker, vacum, clear_selection, handle,
                          menu_operation_handler;
};

int main (int arg, char** argv) {
    rclcpp::init(arg, argv);
    auto node = std::make_shared<Renderer>();
    node -> init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}