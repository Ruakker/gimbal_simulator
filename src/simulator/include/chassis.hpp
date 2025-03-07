#include "interfaces/srv/chassis_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// clang-format off
const double centerX     = 2,
             centerY     = 0,
             centerZ     = 0,
             velocityX   = 0.01,
             velocityY   = 0.00,
             velocityZ   = 0,
             velocityAng = 0.01,
             border      = 3;
// clang-format on

class ChassisNode : public rclcpp::Node {
   public:
    ChassisNode();

   private:
    double centerX_, centerY_, centerZ_, Ang_;
    double velocityX_, velocityY_, velocityZ_;
    double velocityAng_;
    double border_;
    int directionX_, directionY_, directionZ_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> chassisFrameBroadcaster_;

    rclcpp::TimerBase::SharedPtr statusUpdaterTimer_;
    rclcpp::TimerBase::SharedPtr chassisFramePublisherTimer_;

    rclcpp::Service<interfaces::srv::ChassisStatus>::SharedPtr
        chassisStatusModifier_;
};