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
             velocityX   = 0.02,
             velocityY   = 0.01,
             velocityZ   = 0,
             velocityAng = 0.02,
             armorLength = 0.25,  // 125mm
             armorWidth  = 0.27,  // 135mm
             armorHeight = 0.02,
             armorRoll   = 0,
             armorPitch  = M_PI  * 5 / 12,  // 75 degrees
             armorYaw    = 0,
             armorR[]    = {0.50, 0.45},
             armorDz[]   = {0.00, 0.10},
             border      = 3;
// clang-format on

class Armor {
   public:
    Armor(int id, double r, double dz);
    double Dz() const { return dz_; }
    double R() const { return r_; }

   private:
    int id_;
    double r_, dz_;
};

class ChassisNode : public rclcpp::Node {
   public:
    ChassisNode();

   private:
    double centerX_, centerY_, centerZ_, Ang_;
    double velocityX_, velocityY_, velocityZ_;
    double velocityAng_;
    double border_;
    int directionX_, directionY_, directionZ_;
    std::vector<Armor> armors_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> chassisFrameBroadcaster_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        armorPublisher_;

    rclcpp::TimerBase::SharedPtr armorPublisherTimer_;
    rclcpp::TimerBase::SharedPtr statusUpdaterTimer_;
    rclcpp::TimerBase::SharedPtr chassisFramePublisherTimer_;

    rclcpp::Service<interfaces::srv::ChassisStatus>::SharedPtr
        chassisStatusModifier_;
};