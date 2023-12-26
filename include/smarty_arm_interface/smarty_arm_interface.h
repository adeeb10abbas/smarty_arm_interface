#ifndef SMARTY_ARM_NODE_H
#define SMARTY_ARM_NODE_H

/* C++ headers */
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

/* ROS2 headers */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "smarty_arm_interface/msg/ptipacket.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "smarty_arm_interface/shm.h"
/* C headers */
extern "C" {
#include "shm.h"
};

class SMARTY_ARM_Node : public rclcpp::Node {
public:
    explicit SMARTY_ARM_Node(const std::string& name, Arm* arm, const std::string& type);

    // ~SMARTY_ARM_Node() override;

    void run();

private:
    rclcpp::Subscription<smarty_arm_interface::msg::Ptipacket>::SharedPtr smarty_arm_packet_sub;
    rclcpp::Publisher<smarty_arm_interface::msg::Ptipacket>::SharedPtr smarty_arm_packet_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr smarty_arm_pose_pub;

    Arm* arm;
    std::string node_type;

    void publish_ptipacket();
    void publish_pose_state();
    void ptipacket_callback(const smarty_arm_interface::msg::Ptipacket::SharedPtr msg);

    // Replace ROS services with ROS2 services if needed
    // bool initSlave(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &res);
};

#endif /* SMARTY_ARM_NODE_H */
