#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "buddy_interfaces/msg/body_position.hpp"
#include <cmath>
#include <map>
#include <string>
#include <vector>

class DualArmJointPublisher : public rclcpp::Node {
public:
    DualArmJointPublisher() : Node("body_joint_publisher") {
        joint_limits_["joint_2"] = std::make_pair(-0.5235, 0.5235);  
        
        joint_limits_["joint_5"] = std::make_pair(-0.7853, 1.5708);
        joint_limits_["joint_6"] = std::make_pair(0.0, 1.0472);
        joint_limits_["joint_7"] = std::make_pair(-0.7853, 0.7853);
        joint_limits_["joint_8"] = std::make_pair(0.1745, 1.5708);
        
        joint_limits_["joint_9"] = std::make_pair(-0.7853, 1.5708);
        joint_limits_["joint_10"] = std::make_pair(0.0, 1.0472);
        joint_limits_["joint_11"] = std::make_pair(-0.7853, 0.7853);
        joint_limits_["joint_12"] = std::make_pair(0.1745, 1.5708);
        
        std::vector<std::string> right_joints = {"joint_5", "joint_6", "joint_7", "joint_8"};
        std::vector<std::string> left_joints = {"joint_9", "joint_10", "joint_11", "joint_12"};
        
        last_right_pos_ = calculate_midpoints(right_joints);
        last_left_pos_ = calculate_midpoints(left_joints);
        
        torso_tilt_ = 0.0;
        
        subscription_ = this->create_subscription<buddy_interfaces::msg::BodyPosition>(
            "body_tracker", 10, 
            std::bind(&DualArmJointPublisher::arm_tracker_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&DualArmJointPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Dual arm joint publisher initialized");
    }

private:
    const float MAX_CHANGE_PER_STEP = 0.5;

    std::map<std::string, float> calculate_midpoints(const std::vector<std::string>& joints) {
        std::map<std::string, float> result;
        for (const auto& j : joints) {
            result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0;
        }
        return result;
    }
    
    std::map<std::string, float> process_arm_data(const std::array<float, 4>& angles, 
                                              const std::vector<std::string>& arm_joints, 
                                              const bool is_right) {
        std::map<std::string, float> positions;
        
        positions[arm_joints[0]] = angles[0] * M_PI / 180.0;  
        positions[arm_joints[1]] = angles[1] * M_PI / 180.0;  
        positions[arm_joints[2]] = angles[2] * M_PI / 180.0;  
        positions[arm_joints[3]] = angles[3] * M_PI / 180.0;  
        
        for (const auto& joint : arm_joints) {
            float lower = joint_limits_[joint].first;
            float upper = joint_limits_[joint].second;
            positions[joint] = std::max(lower, std::min(upper, positions[joint]));
        }

        for (const auto& joint : arm_joints) {
            float target = positions[joint];
            float current = is_right ? last_right_pos_[joint] : last_left_pos_[joint];
            float delta = target - current;

            if (std::abs(delta) > MAX_CHANGE_PER_STEP) {
                delta = (delta > 0) ? MAX_CHANGE_PER_STEP : -MAX_CHANGE_PER_STEP;
                target = current + delta;
            }
            
            positions[joint] = target;
        }
        
        return positions;
    }
    
    void arm_tracker_callback(const buddy_interfaces::msg::BodyPosition::SharedPtr msg) {
        if (!msg->is_valid) {
            RCLCPP_INFO(this->get_logger(), "Datos inválidos. Manteniendo posición.");
            return;
        }

        float tilt_angle = msg->shoulder_tilt_angle > 0 ? msg->shoulder_tilt_angle - 180 : msg->shoulder_tilt_angle + 180;
        
        float lower = joint_limits_["joint_2"].first;
        float upper = joint_limits_["joint_2"].second;
        float tilt_angle_radians = static_cast<float>(tilt_angle * M_PI / 180.0);
        torso_tilt_ = std::max(lower, std::min(upper, tilt_angle_radians));
                
        std::array<float, 4> right_angles = {
            msg->right_shoulder_elbow_zy,
            msg->right_shoulder_elbow_yx,
            msg->right_elbow_wrist_yx,
            msg->right_elbow_wrist_zy
        };
        
        std::vector<std::string> right_joints = {"joint_5", "joint_6", "joint_7", "joint_8"};
        last_right_pos_ = process_arm_data(right_angles, right_joints, true);
        
        std::array<float, 4> left_angles = {
            msg->left_shoulder_elbow_zy,
            msg->left_shoulder_elbow_yx,
            msg->left_elbow_wrist_yx,
            msg->left_elbow_wrist_zy
        };
        
        std::vector<std::string> left_joints = {"joint_9", "joint_10", "joint_11", "joint_12"};
        last_left_pos_ = process_arm_data(left_angles, left_joints, false);
        
    }
    
    void timer_callback() {
        auto joint_state = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state->header.stamp = this->now();
        
        for (int i = 1; i <= 12; i++) {
            joint_state->name.push_back("joint_" + std::to_string(i));
        }
        
        joint_state->position = {
            0.0,
            torso_tilt_,
            0.0, 0.0,
            last_right_pos_["joint_5"],
            last_right_pos_["joint_6"],
            last_right_pos_["joint_7"],
            0.87265, 
            last_left_pos_["joint_9"],
            last_left_pos_["joint_10"],
            last_left_pos_["joint_11"],
            0.87265
        };
        
        publisher_->publish(*joint_state);
        
        static rclcpp::Time last_log_time = this->now();
        if ((this->now() - last_log_time).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "Posiciones publicadas para torso y brazos");
            last_log_time = this->now();
        }
    }
    
    std::map<std::string, std::pair<float, float>> joint_limits_;
    std::map<std::string, float> last_right_pos_;
    std::map<std::string, float> last_left_pos_;
    float torso_tilt_;
    
    rclcpp::Subscription<buddy_interfaces::msg::BodyPosition>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmJointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}