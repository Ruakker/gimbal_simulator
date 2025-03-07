#include "armor.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

Armor::Armor(int id, double r, double dz) : id_(id), r_(r), dz_(dz) {}

ArmorNode::ArmorNode() : Node("armor_node") {
    for (int i = 0; i < 4; ++i)
        armors_.emplace_back(i, armorR[i & 1], armorDz[i & 1]);

    armorPublisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("armors",
                                                                     10);

    armorPublisherTimer_ =
        this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            // RCLCPP_INFO(this->get_logger(), "Published armors");
            auto markerArray = visualization_msgs::msg::MarkerArray();
            for (int i = 0; i < 4; ++i) {
                // RCLCPP_INFO(this->get_logger(), "id: %d", i);
                visualization_msgs::msg::Marker armor;
                armor.header.frame_id = "armor" + std::to_string(i);
                armor.header.stamp = this->now();
                armor.ns = "armor";
                armor.id = i;
                armor.type = visualization_msgs::msg::Marker::CUBE;
                armor.action = visualization_msgs::msg::Marker::ADD;

                armor.pose.position.x = 0;
                armor.pose.position.y = 0;
                armor.pose.position.z = 0;

                armor.scale.x = armorLength;
                armor.scale.y = armorWidth;
                armor.scale.z = armorHeight;

                armor.color.r = 1.0;
                armor.color.g = 0.0;
                armor.color.b = 0.0;
                armor.color.a = 1.0;

                armor.lifetime = rclcpp::Duration(0, 0);

                markerArray.markers.push_back(armor);
            }
            // RCLCPP_INFO(this->get_logger(), "Publishing armors");
            armorPublisher_->publish(markerArray);
        });

    armorFrameBroadcasterTimer_ =
        this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            for (int i = 0; i < 4; ++i) {
                auto armor = geometry_msgs::msg::TransformStamped();
                armor.header.stamp = this->now();
                armor.header.frame_id = "chassis";
                armor.child_frame_id = "armor" + std::to_string(i);
                armor.transform.translation.x =
                    armors_[i].R() * cos(i * M_PI / 2);
                armor.transform.translation.y =
                    armors_[i].R() * sin(i * M_PI / 2);
                armor.transform.translation.z = armors_[i].Dz();
                tf2::Quaternion rotation;
                rotation.setRPY(armorRoll, armorPitch, armorYaw + i * M_PI / 2);
                armor.transform.rotation.x = rotation.x();
                armor.transform.rotation.y = rotation.y();
                armor.transform.rotation.z = rotation.z();
                armor.transform.rotation.w = rotation.w();
                armorFrameBroadcaster_->sendTransform(armor);
            }
        });

    armorFrameBroadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Chassis node started");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorNode>());
    rclcpp::shutdown();
    return 0;
}
