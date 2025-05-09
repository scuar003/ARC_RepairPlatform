/*  voxel_mapper.cpp  --------------------------------------------------------

   Incremental 3‑D reconstruction server:
   Publishes the voxel‑centroid map ONLY when the “publish_active” flag
   is true.  Flag can be toggled at run‑time with the SetBool service
   ~/set_publish_active.

   ROS 2 Humble Hawksbill
---------------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>
#include <queue>
#include <mutex>
#include <chrono>

using namespace std::chrono_literals;

/* ------------------------------------------------------------------------ */
/*  Type aliases                                                            */
/* ------------------------------------------------------------------------ */
using PointT     = pcl::PointXYZRGB;
using CloudT     = pcl::PointCloud<PointT>;
using CloudPtr   = CloudT::Ptr;
using CloudConst = CloudT::ConstPtr;
using MsgT       = sensor_msgs::msg::PointCloud2;

/* ------------------------------------------------------------------------ */
/*  VoxelMapper class                                                       */
/* ------------------------------------------------------------------------ */
class VoxelMapper : public rclcpp::Node
{
public:
  VoxelMapper()
  : Node("voxel_mapper"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    map_octree_(voxel_resolution_)
  {
    /* ---------------- Parameters ---------------- */
    declare_parameter<std::string>("input_topic", "/point_cloud");
    declare_parameter<std::string>("target_frame", "base_link");
    declare_parameter<double>("voxel_size",   0.02);   // m
    declare_parameter<int>("sor_k",           24);
    declare_parameter<double>("sor_stddev",   1.5);
    declare_parameter<bool>("use_radius",     false);
    declare_parameter<int>("ror_k",           16);
    declare_parameter<double>("ror_radius",   0.05);   // m
    declare_parameter<double>("publish_hz",   3.0);
    declare_parameter<bool>("publish_active", false);  // <-- NEW

    input_topic_       = get_parameter("input_topic").as_string();
    target_frame_      = get_parameter("target_frame").as_string();
    voxel_resolution_  = get_parameter("voxel_size").as_double();
    sor_k_             = get_parameter("sor_k").as_int();
    sor_stddev_        = get_parameter("sor_stddev").as_double();
    use_radius_        = get_parameter("use_radius").as_bool();
    ror_k_             = get_parameter("ror_k").as_int();
    ror_radius_        = get_parameter("ror_radius").as_double();
    publish_enabled_   = get_parameter("publish_active").as_bool(); // NEW

    map_octree_ = OctreeT(voxel_resolution_);

    /* ---------------- I/O ----------------------- */
    auto qos = rclcpp::SensorDataQoS();

    sub_ = create_subscription<MsgT>(
        input_topic_, qos,
        std::bind(&VoxelMapper::enqueueCloud, this, std::placeholders::_1));

    pub_ = create_publisher<MsgT>("processed_points", 1);

    /* ---------------- Timers -------------------- */
    worker_timer_ = create_wall_timer(33ms,
        std::bind(&VoxelMapper::processQueue, this));

    double pub_period = 1.0 /
        std::max(1.0, get_parameter("publish_hz").as_double());

    pub_timer_ = create_wall_timer(
        std::chrono::duration<double>(pub_period),
        std::bind(&VoxelMapper::publishMap, this));

    /* ---------------- Service ------------------- */
    srv_ = create_service<std_srvs::srv::SetBool>(
        "set_publish_active",
        std::bind(&VoxelMapper::handleSetPublish, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
        "VoxelMapper ready — listening on %s, publish_active=%s",
        input_topic_.c_str(),
        publish_enabled_ ? "true" : "false");
  }

private:
  /* ---------------------------------  Data  -------------------------------- */
  struct Job { MsgT::ConstSharedPtr msg; };

  /* Parameters & state */
  std::string input_topic_;
  std::string target_frame_;
  double voxel_resolution_{0.02};
  int    sor_k_;
  double sor_stddev_;
  bool   use_radius_;
  int    ror_k_;
  double ror_radius_;
  bool   publish_enabled_{false};                     // <-- NEW

  /* ROS interfaces */
  rclcpp::Subscription<MsgT>::SharedPtr sub_;
  rclcpp::Publisher<MsgT>::SharedPtr    pub_;
  rclcpp::TimerBase::SharedPtr          worker_timer_;
  rclcpp::TimerBase::SharedPtr          pub_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;   // NEW

  /* TF */
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /* Thread‑safe job queue */
  std::mutex            queue_mutex_;
  std::queue<Job>       job_queue_;

  /* Global voxel map */
  using OctreeT = pcl::octree::OctreePointCloudVoxelCentroid<PointT>;
  OctreeT map_octree_;

  /* ---------------------------  Callbacks ----------------------------------*/
  void enqueueCloud(const MsgT::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    job_queue_.push({msg});
  }

  void processQueue()
  {
    std::queue<Job> local;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      std::swap(local, job_queue_);
    }

    while (!local.empty()) {
      auto job = local.front(); local.pop();

      CloudPtr cloud(new CloudT);
      pcl::fromROSMsg(*job.msg, *cloud);

      if (!transformToTarget(cloud, job.msg->header.frame_id))
        continue;

      CloudPtr clean(new CloudT);
      filterCloud(cloud, clean);
      integrate(clean);
    }
  }

  void publishMap()
  {
    if (!publish_enabled_) return;                 // <-- NEW guard

    std::vector<PointT, Eigen::aligned_allocator<PointT>> vox_centroids;
    map_octree_.getVoxelCentroids(vox_centroids);

    if (vox_centroids.empty()) return;

    CloudPtr cloud(new CloudT);
    cloud->points = std::move(vox_centroids);
    cloud->width  = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    MsgT msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = target_frame_;
    msg.header.stamp    = now();
    pub_->publish(msg);
  }

  /* ---------------- Service callback --------------------------------------*/
  void handleSetPublish(
      const std::shared_ptr<std_srvs::srv::SetBool::Request>  req,
      std::shared_ptr<std_srvs::srv::SetBool::Response>       res)
  {
    publish_enabled_ = req->data;
    res->success = true;
    res->message = publish_enabled_ ?
        "Publishing enabled" : "Publishing disabled";

    RCLCPP_INFO(get_logger(), "Set publish_active -> %s",
                publish_enabled_ ? "true" : "false");
  }

  /* --------------------------  Helpers  ----------------------------------*/
  bool transformToTarget(CloudPtr &cloud, const std::string &src_frame)
  {
    if (src_frame == target_frame_) return true;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(target_frame_, src_frame,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF lookup from %s to %s failed: %s",
          src_frame.c_str(), target_frame_.c_str(), ex.what());
      return false;
    }

    Eigen::Matrix4f tf_f = tf2::transformToEigen(tf).matrix().cast<float>();
    pcl::transformPointCloud(*cloud, *cloud, tf_f);
    return true;
  }

  void filterCloud(const CloudConst &in, CloudPtr &out)
  {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(in);
    sor.setMeanK(sor_k_);
    sor.setStddevMulThresh(sor_stddev_);
    CloudPtr tmp(new CloudT);
    sor.filter(*tmp);

    if (use_radius_) {
      pcl::RadiusOutlierRemoval<PointT> ror;
      ror.setInputCloud(tmp);
      ror.setMinNeighborsInRadius(ror_k_);
      ror.setRadiusSearch(ror_radius_);
      CloudPtr tmp2(new CloudT);
      ror.filter(*tmp2);
      tmp.swap(tmp2);
    }

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(tmp);
    vg.setLeafSize(voxel_resolution_,
                   voxel_resolution_,
                   voxel_resolution_);
    vg.filter(*out);
  }

  void integrate(const CloudConst &clean)
  {
    map_octree_.setInputCloud(clean);
    map_octree_.addPointsFromInputCloud();
  }
};

/* ------------------------------------------------------------------------ */
/*  main                                                                    */
/* ------------------------------------------------------------------------ */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<VoxelMapper>();
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
