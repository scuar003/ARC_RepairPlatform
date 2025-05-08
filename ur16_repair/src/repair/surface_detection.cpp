#include <ur16_repair/repair/repair_operations.hpp>

using namespace repairs;

RepairOperations::RepairOperations(std::shared_ptr<tf2_ros::Buffer> tf_buffer, const std::string& target_frame)
    : tf_buffer_(tf_buffer), target_frame_(target_frame) {}


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
