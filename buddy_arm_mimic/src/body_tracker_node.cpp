#include "rclcpp/rclcpp.hpp"
#include "buddy_interfaces/msg/body_points.hpp"
#include "buddy_interfaces/msg/body_position.hpp"
#include <cmath>

class BodyTrackerNode : public rclcpp::Node {
public:
    BodyTrackerNode() : Node("body_tracker_node") {
        subscription_ = this->create_subscription<buddy_interfaces::msg::BodyPoints>(
            "body_points", 10, 
            std::bind(&BodyTrackerNode::body_points_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<buddy_interfaces::msg::BodyPosition>("body_tracker", 10);
        
        RCLCPP_INFO(this->get_logger(), "Body tracker node initialized");
    }

private:
    bool last_detection_valid = false;
    buddy_interfaces::msg::BodyPosition last_valid_arm_msg;

    float smooth_angle(float new_angle, float prev_angle, float alpha = 0.2) {
        return alpha * new_angle + (1 - alpha) * prev_angle;
    }

    float calculate_angle_with_vertical(float shoulder_x, float shoulder_y, float elbow_x, float elbow_y) {
        float v_x = elbow_x - shoulder_x;
        float v_y = elbow_y - shoulder_y;
        
        return atan2(-v_x, v_y) * 180.0 / M_PI;
    }
    
    float calculate_shoulder_tilt(float left_shoulder_x, float left_shoulder_y,
                               float right_shoulder_x, float right_shoulder_y) {
        float dx = right_shoulder_x - left_shoulder_x;
        float dy = right_shoulder_y - left_shoulder_y;
        
        return atan2(dy, dx) * 180.0 / M_PI;
    }
    
    float calculate_relative_angle(float shoulder_x, float shoulder_y,
                               float elbow_x, float elbow_y,
                               float wrist_x, float wrist_y) {
        float v_x = wrist_x - elbow_x;
        float v_y = wrist_y - elbow_y;
        float u_x = elbow_x - shoulder_x;
        float u_y = elbow_y - shoulder_y;
        
        float det_v_u = u_x * v_y - u_y * v_x;
        float dot_v_u = u_x * v_x + u_y * v_y;
        
        return atan2(det_v_u, dot_v_u) * 180.0 / M_PI;
    }
    
    void body_points_callback(const buddy_interfaces::msg::BodyPoints::SharedPtr msg) {
        buddy_interfaces::msg::BodyPosition arm_msg;
        arm_msg.is_valid = false;

        if (!msg->is_detected) {
            if (last_detection_valid) {
                last_valid_arm_msg.is_valid = false;
                publisher_->publish(last_valid_arm_msg);
                RCLCPP_WARN(this->get_logger(), "No detection! Using last valid angles but marking as invalid.");
            }
            last_detection_valid = false;
            return;
        }
        
        float shoulder_tilt = -calculate_shoulder_tilt(
            msg->left_shoulder_x, msg->left_shoulder_y,
            msg->right_shoulder_x, msg->right_shoulder_y
        );
        
        float angle_shoulder_right_elbow_YX = calculate_angle_with_vertical(
            msg->right_shoulder_x, msg->right_shoulder_y,
            msg->right_elbow_x, msg->right_elbow_y
        );
        
        float angle_elbow_right_wrist_YX = -calculate_relative_angle(
            msg->right_shoulder_x, msg->right_shoulder_y,
            msg->right_elbow_x, msg->right_elbow_y,
            msg->right_wrist_x, msg->right_wrist_y
        );
        
        float angle_shoulder_left_elbow_YX = -calculate_angle_with_vertical(
            msg->left_shoulder_x, msg->left_shoulder_y,
            msg->left_elbow_x, msg->left_elbow_y
        );
        
        float angle_elbow_left_wrist_YX = -calculate_relative_angle(
            msg->left_shoulder_x, msg->left_shoulder_y,
            msg->left_elbow_x, msg->left_elbow_y,
            msg->left_wrist_x, msg->left_wrist_y
        );
        
        arm_msg.shoulder_tilt_angle = shoulder_tilt;
        
        arm_msg.right_shoulder_elbow_zy = 0.0;  
        arm_msg.right_elbow_wrist_zy = 0.0;
        arm_msg.right_shoulder_elbow_yx = angle_shoulder_right_elbow_YX;
        arm_msg.right_elbow_wrist_yx = angle_elbow_right_wrist_YX;
        arm_msg.right_wrist_x = msg->right_wrist_x;
        arm_msg.right_wrist_y = msg->right_wrist_y;
        
        arm_msg.left_shoulder_elbow_zy = 0.0;  
        arm_msg.left_elbow_wrist_zy = 0.0;
        arm_msg.left_shoulder_elbow_yx = angle_shoulder_left_elbow_YX;
        arm_msg.left_elbow_wrist_yx = angle_elbow_left_wrist_YX;
        arm_msg.left_wrist_x = msg->left_wrist_x;
        arm_msg.left_wrist_y = msg->left_wrist_y;

        if (last_detection_valid) {
            arm_msg.right_shoulder_elbow_yx = smooth_angle(arm_msg.right_shoulder_elbow_yx, last_valid_arm_msg.right_shoulder_elbow_yx);
            arm_msg.right_elbow_wrist_yx = smooth_angle(arm_msg.right_elbow_wrist_yx, last_valid_arm_msg.right_elbow_wrist_yx);
            arm_msg.left_shoulder_elbow_yx = smooth_angle(arm_msg.left_shoulder_elbow_yx, last_valid_arm_msg.left_shoulder_elbow_yx);
            arm_msg.left_elbow_wrist_yx = smooth_angle(arm_msg.left_elbow_wrist_yx, last_valid_arm_msg.left_elbow_wrist_yx);
        }

        last_valid_arm_msg = arm_msg;
        last_detection_valid = true;
        arm_msg.is_valid = true;
        
        publisher_->publish(arm_msg);
        
        RCLCPP_INFO(this->get_logger(), "Sending body position message...");
    }
    
    rclcpp::Subscription<buddy_interfaces::msg::BodyPoints>::SharedPtr subscription_;
    rclcpp::Publisher<buddy_interfaces::msg::BodyPosition>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}