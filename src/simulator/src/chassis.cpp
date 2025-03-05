#include "chassis.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

Armor::Armor(int id, double r, double dz) : id_(id), r_(r), dz_(dz) {}

ChassisNode::ChassisNode() : Node("chassis_node") {
    centerX_ = centerX;
    centerY_ = centerY;
    centerZ_ = centerZ;
    velocityX_ = velocityX;
    velocityY_ = velocityY;
    velocityZ_ = velocityZ;
    velocityAng_ = velocityAng;
    Ang_ = 0;
    directionX_ = directionY_ = directionZ_ = 1;
    border_ = border;
    for (int i = 0; i < 4; ++i)
        armors_.emplace_back(i, armorR[i & 1], armorDz[i & 1]);

    // Modify chassis's staus by receiving a request from client
    chassisStatusModifier_ =
        this->create_service<interfaces::srv::ChassisStatus>(
            "chassis_status_modifier",
            [this](
                const std::shared_ptr<interfaces::srv::ChassisStatus::Request>
                    request,
                std::shared_ptr<interfaces::srv::ChassisStatus::Response>
                    response) {
                // RCLCPP_INFO(this->get_logger(), "Modified chassis status");
                centerX_ = request->center_x;
                centerY_ = request->center_y;
                centerZ_ = request->center_z;
                velocityX_ = request->velocity_x;
                velocityY_ = request->velocity_y;
                velocityZ_ = request->velocity_z;
                velocityAng_ = request->velocity_ang;
                response->status = true;
            });

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
                armor.header.frame_id = "chassis";
                armor.header.stamp = this->now();
                armor.ns = "armor";
                armor.id = i;
                armor.type = visualization_msgs::msg::Marker::CUBE;
                armor.action = visualization_msgs::msg::Marker::ADD;

                armor.pose.position.x = armors_[i].R() * cos(i * M_PI / 2);
                armor.pose.position.y = armors_[i].R() * sin(i * M_PI / 2);
                armor.pose.position.z = centerZ_ + armors_[i].Dz();

                tf2::Quaternion rotation;
                rotation.setRPY(armorRoll, armorPitch, armorYaw + i * M_PI / 2);
                armor.pose.orientation.x = rotation.x();
                armor.pose.orientation.y = rotation.y();
                armor.pose.orientation.z = rotation.z();
                armor.pose.orientation.w = rotation.w();

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

    statusUpdaterTimer_ =
        this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            // RCLCPP_INFO(this->get_logger(), "Updated chassis status");
            Ang_ += velocityAng_;
            while (Ang_ > 2 * M_PI) Ang_ -= 2 * M_PI;
            centerX_ += velocityX_ * directionX_;
            centerY_ += velocityY_ * directionY_;
            centerZ_ += velocityZ_ * directionZ_;
            if (centerX_ > border_ || centerX_ < -border_) directionX_ *= -1;
            if (centerY_ > border_ || centerY_ < -border_) directionY_ *= -1;
            if (centerZ_ > border_ || centerZ_ < -border_) directionZ_ *= -1;
        });

    chassisFramePublisherTimer_ =
        this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            auto transformStamped = geometry_msgs::msg::TransformStamped();
            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id = "map";
            transformStamped.child_frame_id = "chassis";
            transformStamped.transform.translation.x = centerX_;
            transformStamped.transform.translation.y = centerY_;
            transformStamped.transform.translation.z = centerZ_;
            tf2::Quaternion rotation;
            rotation.setRPY(0, 0, Ang_);
            transformStamped.transform.rotation.x = rotation.x();
            transformStamped.transform.rotation.y = rotation.y();
            transformStamped.transform.rotation.z = rotation.z();
            transformStamped.transform.rotation.w = rotation.w();
            chassisFrameBroadcaster_->sendTransform(transformStamped);
        });

    chassisFrameBroadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(this->get_logger(), "Chassis node started");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisNode>());
    rclcpp::shutdown();
    return 0;
}