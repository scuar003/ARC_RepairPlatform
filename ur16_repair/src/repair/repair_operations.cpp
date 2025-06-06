#include <ur16_repair/repair/repair_operations.hpp>
#include <iostream>

namespace repairs {

    RepairOperations::RepairOperations() {}

    RepairOperations::RepairOperations(std::shared_ptr<tf2_ros::Buffer> tf_buffer, 
                                       const std::string& target_frame,
                                       ToolRequestFn tool_request) : target_frame_{target_frame}, tf_buffer_{std::move(tf_buffer)}, 
                                                                     tool_request_{std::move(tool_request)} {}

    void RepairOperations::connect(const std::string &rb_ip) {
        if(robot_ && robot_->isConnected()) return;
        std::cout << "Connecting to robot at: " << rb_ip << std::endl;
        robot_ = std::make_unique<RTDEControlInterface>(rb_ip);
    }

    void RepairOperations::scanEnv(const std::string &rb_ip){
        connect(rb_ip);
        //Start
        moveHome(rb_ip);
        //right
        robot_->moveJ({-3.25, -0.94, -1.33, -1.65, 1.55, 0.0}, rb_acc, rb_vel);
        //left
        robot_->moveJ({0.83, -0.94, -1.33, -1.65, 1.55, 0.0}, rb_acc, rb_vel);
        //end
        moveHome(rb_ip);
    }

    void RepairOperations::moveHome(const std::string &rb_ip) {
        connect(rb_ip);
        robot_->moveJ({-3.08, -0.94, -1.33, -1.65, 1.55, 0.0}, rb_acc + 0.1  , rb_vel + 0.5);
    }

    void RepairOperations::grindArea(const std::string &rb_ip, const std::vector<CusEigen> &area){
        getGrinder(rb_ip);
        std::cout << "grinder has been picked, moving home now " << std::endl;

        robot_->setPayload(2.2, {-0.008,-0.02,0.062});
        robot_->setTcp({0.00344,0.13178,0.25004,0.0001,3.0982,0.4994});
        

        moveHome(rb_ip);
        std::cout << " Starting Grinding procces... " << std::endl;
        if(area.size() < 4) {
            std::cerr << "Need Area of operation " << std::endl;
            return;
        }
        std::vector<Vec3> corner;
        for (int i = 0; i < 4; ++i) {
            const auto &p = area[i];
            corner.emplace_back(-p.x, -p.y, p.z);
        }   
        auto path = buildPath(corner, 0.01, 0.01, 2, 0.005);
        printPath(path);
        executePath(path, 0.1, 0.1);
        path.clear();
        returnGrinder(rb_ip);

    }

    void RepairOperations::getGrinder(const std::string &rb_ip) {
        moveHome(rb_ip);

        robot_->setTcp({0,0,0,0,0,0});
        robot_->moveJ({deg2rad(6.22), deg2rad(-90.27), deg2rad(-48.50),
                      deg2rad(-131.23), deg2rad(88.76), deg2rad(-83.74)}, rb_acc + 0.12, rb_vel + 0.12 );
        //tool request to unlock 
        if(!tool_request_("tool_unlock"))
            throw std::runtime_error("Failed to unlock tool-changer");
        robot_->moveL({0.412, -0.133, 0.450, 3.121, -0.007, -0.044}, rb_acc + 0.1, rb_vel + 0.1);
        robot_->moveL({0.412, -0.133, 0.41834, 3.121, -0.007, -0.044}, rb_acc, rb_vel);
        //tool_request to lock
        if (!tool_request_("tool_lock"))
            throw std::runtime_error("Failed to lock the tool-changer");
        
        robot_->moveL({0.412, -0.133, 0.450, 3.121, -0.007, -0.044}, rb_acc + 0.1, rb_vel + 0.1);
        robot_->moveL({0.412, -0.133, 0.7 , 3.121, -0.007, -0.044}, rb_acc, rb_vel);

    }
    
    void RepairOperations::returnGrinder(const std::string &ip) {
        robot_->moveJ({deg2rad(6.22), deg2rad(-90.27), deg2rad(-48.50),
                    deg2rad(-131.23), deg2rad(88.76), deg2rad(-83.74)}, rb_acc + 0.12, rb_vel + 0.12 );
        robot_->setTcp({0,0,0,0,0,0});
        robot_->moveL({0.412, -0.133, 0.450, 3.121, -0.007, -0.044}, rb_acc + 0.1, rb_vel + 0.1);
        robot_->moveL({0.412, -0.133, 0.41834, 3.121, -0.007, -0.044}, rb_acc, rb_vel);
        if(!tool_request_("tool_unlock"))
            throw std::runtime_error("Failed to unlock tool-changer");
        
        robot_->moveL({0.412, -0.133, 0.450, 3.121, -0.007, -0.044}, rb_acc + 0.1, rb_vel + 0.1);
        robot_->moveL({0.412, -0.133, 0.7 , 3.121, -0.007, -0.044}, rb_acc, rb_vel); 
        moveHome(ip);
    }              

/*------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------*/
/*-------------------------------CLEANING---------------------------------------------*/

void RepairOperations::clean(const std::string &rb_ip, const std::vector<CusEigen> &area) {}

/*------------------------------------------------------------------------------------*/

/*-----------------HELPERS ----------------------- */

    PoseArray RepairOperations::detect(const PointCloud2::SharedPtr cloud_msg_) {
    PoseArray pose_array;
    pose_array.header.frame_id = target_frame_;

    if (!cloud_msg_) {
        std::cerr << "[RepairOperations] Received null cloud." << std::endl;
        return pose_array;  // Return empty
    }

    auto cloud_o3d = std::make_shared<open3d::geometry::PointCloud>();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg_, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
            cloud_o3d->points_.emplace_back(*iter_x, *iter_y, *iter_z);
        }
    }

    cloud_o3d->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));

    auto planes = cloud_o3d->DetectPlanarPatches(30.0, 85.0, 1.0, 0.05, 0, open3d::geometry::KDTreeSearchParamKNN(30));
    if (planes.empty()) {
        std::cerr << "[RepairOperations] No planes detected." << std::endl;
        return pose_array;
    }

    pose_array.header.stamp = cloud_msg_->header.stamp;

    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(target_frame_, cloud_msg_->header.frame_id, cloud_msg_->header.stamp);
    } catch (const tf2::TransformException &ex) {
        std::cerr << "[RepairOperations] TF Lookup failed: " << ex.what() << std::endl;
        return pose_array;
    }

    for (const auto &plane : planes) {
        const Eigen::Vector3d &center = plane->center_;
        const Eigen::Matrix3d &R = plane->R_;
        const Eigen::Vector3d &extent = plane->extent_;

        int min_idx;
        extent.minCoeff(&min_idx);
        std::vector<int> idx = {0, 1, 2};
        idx.erase(std::remove(idx.begin(), idx.end(), min_idx), idx.end());

        for (int i : {-1, 1}) {
            for (int j : {-1, 1}) {
                Eigen::Vector3d offset = Eigen::Vector3d::Zero();
                offset[idx[0]] = extent[idx[0]] / 2.0 * i;
                offset[idx[1]] = extent[idx[1]] / 2.0 * j;
                offset[min_idx] = 0;

                Eigen::Vector3d corner = center + R * offset;

                tf2::Vector3 tf_corner(corner.x(), corner.y(), corner.z());
                tf2::Vector3 tf_trans = tf2::Transform(tf2::Quaternion(
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w),
                    tf2::Vector3(tf.transform.translation.x,
                                    tf.transform.translation.y,
                                    tf.transform.translation.z)) * tf_corner;

                geometry_msgs::msg::Pose pose;
                pose.position.x = tf_trans.x();
                pose.position.y = tf_trans.y();
                pose.position.z = tf_trans.z();
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0;

                pose_array.poses.push_back(pose);
            }
        }
    }

    std::cout << "[RepairOperations] Planes detected: " << pose_array.poses.size() << " corners." << std::endl;
    return pose_array;
}

    void RepairOperations::printPath(const std::vector<std::vector<double>> &wp) const {
        std::cout << "\n=== Generated moveL sequence (" << wp.size() << " points) ===\n";
        for (const auto &p : wp) {
            std::cout << "robot_->moveL({"
                    << p[0] << ", "
                    << p[1] << ", "
                    << p[2] << ", "
                    << p[3] << ", "
                    << p[4] << ", "
                    << p[5] << "}, rb_acc, rb_vel);" << std::endl;
        }
        std::cout << "=== end sequence ===\n" << std::endl;
    }
    Vec3 RepairOperations::normalFromCorners(const std::vector<Vec3> &c) {
        Vec3 n = (c[2] - c[0]).cross(c[1] - c[0]);
        if (n.z() < 0) n = -n;
        return n.normalized();
    }
    Vec3 RepairOperations::rpyFromNormal(const Vec3 &n) {
        Vec3 z(0,0,1), axis = z.cross(n);
        double angle = std::acos(z.dot(n));
        if (axis.norm() < 1e-6) axis = Vec3::UnitX();
        Eigen::Matrix3d R(Eigen::AngleAxisd(angle, axis.normalized()));
        return { std::atan2(R(2,1),R(2,2)),
                std::asin(-R(2,0)),
                std::atan2(R(1,0),R(0,0)) };
    }
    std::vector<std::vector<double>> RepairOperations::buildPath(const std::vector<Vec3> &c, double grid, double lift, int layers, double step_down) {
        Vec3 op1=c[0], op2=c[1], op3=c[2], op4=c[3];
        Vec3 normal = normalFromCorners({op1,op2,op3});
        auto offs    = [&](const Vec3& p,double d){ return p - d*normal; };

        /* ❷ container for ALL layers */
        std::vector<std::vector<double>> wp;

        /* ❸ loop over layers (= “count” in python) */
        for (int L=0; L<layers; ++L) {
            double lower = step_down * L;               // = lowerDistance*count
            Vec3 p1 = offs(op1, lower);
            Vec3 p2 = offs(op2, lower);
            Vec3 p3 = offs(op3, lower);
            Vec3 p4 = offs(op4, lower);

            normal = normalFromCorners({p1,p2,p3});    // recalc after offset
            Vec3 move  = p2 - p1;
            Vec3 shift = (p4 - p1).normalized()*grid;
            int  passes= static_cast<int>((p4-p1).norm()/grid) + 1;

            /* ❹ actual stripes inside one layer */
            Vec3 cur = p1;
            for (int stripe=0; stripe<passes; ++stripe) {
                Vec3 end = cur + move;

                auto push = [&](const Vec3 &p) {
                    wp.push_back({p.x(), p.y(), p.z(), 0,0,0});   // rx,ry,rz→0
                };

                push(cur);
                push(end);
                push(end + lift*normal);

                if (stripe < passes-1) {
                    Vec3 nextHigh = cur + shift + lift*normal;
                    Vec3 nextLow  = nextHigh - lift*normal;
                    push(nextHigh);
                    push(nextLow);
                    cur = nextLow;
                }
            }
        }
        return wp;
    }
    void RepairOperations::executePath(const std::vector<std::vector<double>> &wp, double acc, double vel) {
        for (const auto &p : wp) 
            robot_->moveL(p, rb_acc, rb_vel);
    }

}