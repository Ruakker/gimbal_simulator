#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// clang-format off
const double armorLength = 0.25,  // 125mm
             armorWidth  = 0.27,  // 135mm
             armorHeight = 0.02,
             armorRoll   = 0,
             armorPitch  = M_PI  * 5 / 12,  // 75 degrees
             armorYaw    = 0,
             armorR[]    = {0.50, 0.45},
             armorDz[]   = {0.00, 0.10};
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

class ArmorNode : public rclcpp::Node {
   public:
    ArmorNode();

   private:
    std::vector<Armor> armors_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        armorPublisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> armorFrameBroadcaster_;

    rclcpp::TimerBase::SharedPtr armorPublisherTimer_;
    rclcpp::TimerBase::SharedPtr armorFrameBroadcasterTimer_;
};
