#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// clang-format off
const double cameraX = 0;
// clang-format on

class CollisionCheckerNode : public rclcpp::Node {
   public:
    CollisionCheckerNode();

   private:
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    rclcpp::TimerBase::SharedPtr collisionCheckerTimer_;
};