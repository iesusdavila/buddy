#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "buddy_interfaces/msg/body_position.hpp"
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <algorithm>

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
        
        last_right_pos_ = calculateMidpoints(right_joints);
        last_left_pos_ = calculateMidpoints(left_joints);
        
        torso_tilt_ = 0.0;
        
        subscription_ = this->create_subscription<buddy_interfaces::msg::BodyPosition>(
            "body_tracker", 10, 
            std::bind(&DualArmJointPublisher::armTrackerCallback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&DualArmJointPublisher::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Dual arm joint publisher initialized");
    }

private:
    std::vector<float> interpolateCubic(float start, float end, size_t steps) {
        std::vector<float> trajectory;
        for (size_t i = 0; i <= steps; ++i) {
            float t = static_cast<float>(i) / steps;
            trajectory.push_back(start + t * (end - start));
        }
        return trajectory;
    }

    std::map<std::string, float> calculateMidpoints(const std::vector<std::string>& joints) {
        std::map<std::string, float> result;
        for (const auto& j : joints) {
            result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0;
        }
        return result;
    }

    float euler2Radian(float degree) {
        return degree * M_PI / 180.0;
    }

    float limitJointPosition(const std::string& joint, float position) {
        float lower = joint_limits_[joint].first;
        float upper = joint_limits_[joint].second;
        return std::max(lower, std::min(upper, position));
    }
    
    std::map<std::string, float> processArmData(const std::array<float, 4>& angles, 
                                              const std::vector<std::string>& arm_joints, 
                                              const bool is_right) {
        std::map<std::string, float> positions;
        
        positions[arm_joints[0]] = euler2Radian(angles[0]);
        positions[arm_joints[1]] = euler2Radian(angles[1]);
        positions[arm_joints[2]] = euler2Radian(angles[2]);
        positions[arm_joints[3]] = euler2Radian(angles[3]);
        
        for (const auto& joint : arm_joints) {
            positions[joint] = limitJointPosition(joint, positions[joint]);
        }

        if (is_right) {
            right_arm_trajectory_.clear();
            for (const auto& joint : arm_joints) {
                float current = last_right_pos_[joint];
                float target = positions[joint];
                right_arm_trajectory_[joint] = interpolateCubic(current, target, INTERPOLATION_STEPS);
            }
        } else {
            left_arm_trajectory_.clear();
            for (const auto& joint : arm_joints) {
                float current = last_left_pos_[joint];
                float target = positions[joint];
                left_arm_trajectory_[joint] = interpolateCubic(current, target, INTERPOLATION_STEPS);
            }
        }
        
        current_step_ = 0;
        return positions;
    }
    
    void armTrackerCallback(const buddy_interfaces::msg::BodyPosition::SharedPtr msg) {
        if (!msg->is_valid) {
            RCLCPP_INFO(this->get_logger(), "Datos inválidos. Manteniendo posición.");
            return;
        }

        float tilt_angle = msg->shoulder_tilt_angle > 0 ? msg->shoulder_tilt_angle - 180 : msg->shoulder_tilt_angle + 180;
        tilt_angle = euler2Radian(tilt_angle);

        torso_tilt_ = limitJointPosition("joint_2", tilt_angle);
                
        std::array<float, 4> right_angles = {
            msg->right_shoulder_elbow_zy,
            msg->right_shoulder_elbow_yx,
            msg->right_elbow_wrist_yx,
            msg->right_elbow_wrist_zy
        };
        
        std::vector<std::string> right_joints = {"joint_5", "joint_6", "joint_7", "joint_8"};
        last_right_pos_ = processArmData(right_angles, right_joints, true);
        
        std::array<float, 4> left_angles = {
            msg->left_shoulder_elbow_zy,
            msg->left_shoulder_elbow_yx,
            msg->left_elbow_wrist_yx,
            msg->left_elbow_wrist_zy
        };
        
        std::vector<std::string> left_joints = {"joint_9", "joint_10", "joint_11", "joint_12"};
        last_left_pos_ = processArmData(left_angles, left_joints, false);
    }
    
    void timerCallback() {
        if (current_step_ < INTERPOLATION_STEPS) {
            for (auto& [joint, trajectory] : right_arm_trajectory_) {
                last_right_pos_[joint] = trajectory[current_step_];
            }
            for (auto& [joint, trajectory] : left_arm_trajectory_) {
                last_left_pos_[joint] = trajectory[current_step_];
            }
            current_step_++;
        }

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

    std::map<std::string, std::vector<float>> right_arm_trajectory_;
    std::map<std::string, std::vector<float>> left_arm_trajectory_;
    size_t current_step_ = 0;
    const size_t INTERPOLATION_STEPS = 10;

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