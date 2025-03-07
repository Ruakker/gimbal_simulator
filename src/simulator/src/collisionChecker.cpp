#include "collisionChecker.hpp"

#include "armor.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define bulletR 0.034  // 17mm

CollisionCheckerNode::CollisionCheckerNode() : Node("collision_checker_node") {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    collisionCheckerTimer_ =
        this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            for (int i = 0; i < 4; ++i) {
                geometry_msgs::msg::TransformStamped bulletPos;
                try {
                    // assume there is only one bullet
                    // haven't finished yet
                    bulletPos = tfBuffer_->lookupTransform(
                        "armor" + std::to_string(i), "bullet",
                        tf2::TimePointZero);
                    // RCLCPP_INFO(this->get_logger(),
                    //             "armor %d, x: %lf, y: %lf, z: %lf", i,
                    //             bulletPos.transform.translation.x,
                    //             bulletPos.transform.translation.y,
                    //             bulletPos.transform.translation.z);
                } catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                }
                double closestX = std::fmax(
                    -armorLength / 2,
                    std::fmin(bulletPos.transform.translation.x, armorLength));
                double closestY = std::fmax(
                    -armorWidth / 2,
                    std::fmin(bulletPos.transform.translation.y, armorWidth));
                double closestZ = std::fmax(
                    -armorHeight / 2,
                    std::fmin(bulletPos.transform.translation.z, armorHeight));

                auto distanceSquared = [](double x1, double y1, double z1,
                                          double x2, double y2, double z2) {
                    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                           (z1 - z2) * (z1 - z2);
                };

                double distSq =
                    distanceSquared(bulletPos.transform.translation.x,
                                    bulletPos.transform.translation.y,
                                    bulletPos.transform.translation.z, closestX,
                                    closestY, closestZ);

                if (distSq <= bulletR * bulletR) {
                    RCLCPP_INFO(this->get_logger(), "Collision detected");
                }
            }
        });
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollisionCheckerNode>());
    rclcpp::shutdown();
    return 0;
}