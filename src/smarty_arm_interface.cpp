#include "smarty_arm_interface/smarty_arm_interface.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace rclcpp;
        

// SMARTY_ARM_Node::SMARTY_ARM_Node(const std::string &name, Arm *armptr, const std::string &type)
//     : Node(name), arm(armptr), node_type(type) {
//     // Initialization and topic subscriptions/publications
//     // Adapt this section to use ROS2 Publisher and Subscriber initialization
// }
SMARTY_ARM_Node::SMARTY_ARM_Node(const std::string& name, Arm* arm, const std::string& type)
: Node(name), arm(arm), node_type(type) {
        // Initialize publishers and subscribers
        if (node_type == "r") {
            smarty_arm_packet_sub = this->create_subscription<smarty_arm_msg::msg::Ptipacket>(
                "/pti_interface_right/pti_output", 1,
                std::bind(&SMARTY_ARM_Node::ptipacket_callback, this, std::placeholders::_1));
            smarty_arm_packet_pub = this->create_publisher<smarty_arm_msg::msg::Ptipacket>(
                "/right_smarty_arm_output", 1);
            smarty_arm_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>(
                "/right_arm_pose", 1);
        } else if (node_type == "l") {
            smarty_arm_packet_sub = this->create_subscription<smarty_arm_msg::msg::Ptipacket>(
                "/pti_interface_left/pti_output", 1,
                std::bind(&SMARTY_ARM_Node::ptipacket_callback, this, std::placeholders::_1));
            smarty_arm_packet_pub = this->create_publisher<smarty_arm_msg::msg::Ptipacket>(
                "/left_smarty_arm_output", 1);
            smarty_arm_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>(
                "/left_arm_pose", 1);
        }

        RCLCPP_INFO(this->get_logger(), "SMARTY_ARM_Node initialized");
}
void SMARTY_ARM_Node::publish_ptipacket() {
    // Publish logic, adapt for ROS2
    /* Publish arm packet through ROS */

    smarty_arm_msg::msg::Ptipacket packet_msg;

    packet_msg.wave.resize(3);
    packet_msg.test.resize(3);

    pthread_mutex_lock(&arm->mutex);
    for (int i = 0; i < 3; i++) {
        packet_msg.wave[i] = arm->ptiPacket[i].wave_out;
        packet_msg.test[i] = arm->ptiPacket[i].test;
    }
    packet_msg.position.x = arm->ee[0].pos;
    packet_msg.position.y = arm->ee[1].pos;
    packet_msg.position.z = arm->ee[2].pos;
    packet_msg.angle.x = arm->ee[3].pos;
    packet_msg.angle.y = arm->ee[4].pos;
    packet_msg.angle.z = arm->ee[5].pos;
    packet_msg.quat.w = arm->quat.w;
    packet_msg.quat.x = arm->quat.x;
    packet_msg.quat.y = arm->quat.y;
    packet_msg.quat.z = arm->quat.z;
    packet_msg.twist.linear.x = arm->ee[0].vel;
    packet_msg.twist.linear.y = arm->ee[1].vel;
    packet_msg.twist.linear.z = arm->ee[2].vel;
    packet_msg.twist.angular.x = arm->ee[3].vel;
    packet_msg.twist.angular.y = arm->ee[4].vel;
    packet_msg.twist.angular.z = arm->ee[5].vel;
    packet_msg.local_stamp = rclcpp::Clock().now().seconds();
    packet_msg.remote_stamp = arm->ts.remote_time;
    pthread_mutex_unlock(&arm->mutex);

    smarty_arm_packet_pub->publish(packet_msg);
}


void SMARTY_ARM_Node::origin_shift_callback(smarty_arm_msg::msg::Config &config){

    pthread_mutex_lock(&arm->mutex);
    origin_position[0] = config.origin_shift_x;
    origin_position[1] = config.origin_shift_y;
    origin_position[2] = config.origin_shift_z;
    pthread_mutex_unlock(&arm->mutex);
    RCLCPP_INFO(this->get_logger(), "Origin shift updated");
}

void SMARTY_ARM_Node::publish_pose_state() {
    geometry_msgs::msg::Pose pose;
    pose.position.x = arm->ee[0].pos;
    pose.position.y = arm->ee[1].pos;
    pose.position.z = arm->ee[2].pos;
    pose.orientation.w = arm->quat.w;
    pose.orientation.x = arm->quat.x;
    pose.orientation.y = arm->quat.y;
    pose.orientation.z = arm->quat.z;

    smarty_arm_pose_pub->publish(pose);

}

void SMARTY_ARM_Node::ptipacket_callback(const smarty_arm_msg::msg::Ptipacket::SharedPtr packet_msg) {
    double delay_time;
    pthread_mutex_lock(&arm->mutex);
    for (int i = 0; i < 3; i++) {
        arm->ptiPacket[i].wave_in = packet_msg->wave[i];
    }

    arm->ptiPacket[0].pos_in = packet_msg->position.x;
    arm->ptiPacket[1].pos_in = packet_msg->position.y;
    arm->ptiPacket[2].pos_in = packet_msg->position.z;

    arm->ptiPacket[0].vel_in = packet_msg->twist.linear.x;
    arm->ptiPacket[1].vel_in = packet_msg->twist.linear.y;
    arm->ptiPacket[2].vel_in = packet_msg->twist.linear.z;

    arm->ptiPacket[0].pos_d_in = packet_msg->position_d.x;
    arm->ptiPacket[1].pos_d_in = packet_msg->position_d.y;
    arm->ptiPacket[2].pos_d_in = packet_msg->position_d.z;

    arm->ts.remote_time = packet_msg->local_stamp;
    delay_time = (rclcpp::Clock().now().seconds() - packet_msg->remote_stamp) / 2.0;
    arm->ts.delay_cycle = (int)(delay_time / 1e-3);
    pthread_mutex_unlock(&arm->mutex);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Write into smarty arm memory, message delay: %lf", delay_time);

}

void SMARTY_ARM_Node::run() {
    int teleop_freq = 1000;
    int low_freq_state_pub = 10;
    int low_freq_state_pub_index = 0;
    rclcpp::Rate loop_rate(teleop_freq);
    arm->ts.remote_time = this->get_clock()->now().seconds();

    while (rclcpp::ok()) {
        /* Publisher (wrap) */
        publish_ptipacket();
        if (low_freq_state_pub_index >= int(teleop_freq/low_freq_state_pub)) {
            low_freq_state_pub_index = 0;
        }
        if (low_freq_state_pub_index == 0) {
            publish_pose_state();
        }
        low_freq_state_pub_index++;

        for(int i = 0; i < DOF/2; i++) {
            arm->ptiPacket[i].position_origin_shift = origin_position[i];
        }

        /* Subscriber callback loop */
        rclcpp::spin_some(shared_from_this());
        loop_rate.sleep();
    }
}


int main(int argc, char** argv) {
    // Check for correct number of arguments
    if (argc < 2 || strlen(argv[1]) != 1) {
        std::cerr << "Usage: " << argv[0] << " [single character, e.g., L or R]" << std::endl;
        return 1;
    }

    // Initialize ROS2
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("Main"), "Launch ROS2 interface");

    // Instantiate input-output data variables
    char armType = argv[1][0]; // Take the first character of argv[1]
    Arm* arm = initArm(armType);
    if (arm == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("Main"), "Init arm failed. shm_open error, errno(%d): %s", errno, strerror(errno));
        return 1;
    }

    // Initialize a node with "master" or "slave" setting
    auto smarty_arm = std::make_shared<SMARTY_ARM_Node>("smarty_arm_interface", arm, std::string(argv[1]));

    for(int i = 0; i < DOF/2; i++) {
        smarty_arm->origin_position[i] = 0.0;
    }

    // TODO: ROS2 parameter handling (if needed, replace dynamic_reconfigure)
    // Set up parameter callback or services as per your application requirements

    RCLCPP_INFO(smarty_arm->get_logger(), "Node starts running");
    // rclcpp::spin(smarty_arm);
    smarty_arm->run();

    rclcpp::shutdown();
    return 0;
}
