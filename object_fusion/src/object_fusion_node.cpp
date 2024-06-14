#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class ObjectFusionNode : public rclcpp::Node
{
public:
    ObjectFusionNode() : Node("object_fusion_node"), proj_coords_received_(false), lidar_received_(false)
    {
        // subs /pub_proj_coords
        proj_coords_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pub_proj_coords", 10, std::bind(&ObjectFusionNode::projCoordsCallback, this, std::placeholders::_1));

        // subs /prcp_obj_list_lidar
        lidar_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/prcp_obj_list_lidar", 10, std::bind(&ObjectFusionNode::lidarCallback, this, std::placeholders::_1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&ObjectFusionNode::checkStatus, this));

        RCLCPP_INFO(this->get_logger(), "ObjectFusionNode node started");
    }

private:
    void projCoordsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        proj_coords_received_ = !msg->data.empty();
        RCLCPP_INFO(this->get_logger(), "Received projected coordinates"); // Processing messages
    }
    void lidarCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        lidar_received_ = !msg->data.empty();
        RCLCPP_INFO(this->get_logger(), "Received lidar object list"); // Processing messages
    }

    void checkStatus()
    {
        if (proj_coords_received_ && lidar_received_)
        {
            RCLCPP_INFO(this->get_logger(), "1: Both topics available  & !()");
        }
        else if (proj_coords_received_ && !lidar_received_)
        {
            RCLCPP_INFO(this->get_logger(), "2: Only projected coordinates");
        }
        else if (!proj_coords_received_ && lidar_received_)
        {
            RCLCPP_INFO(this->get_logger(), "3: Only lidar object");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "4: Neither topic not aviable");
        }

        proj_coords_received_ = false;
        lidar_received_ = false;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr proj_coords_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool proj_coords_received_;
    bool lidar_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
