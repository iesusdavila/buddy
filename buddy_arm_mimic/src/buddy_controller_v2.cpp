#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "buddy_interfaces/msg/body_position.hpp"
#include <cmath>
#include <map>
#include <string>
#include <vector>

struct ArmConfig {
    float L1;
    float L2;
    float scale;
    float Kp;
    float error_threshold;
};

struct Position2D {
    float x;
    float y;
};

Position2D computeForwardKinematics(float theta1, float theta2, const ArmConfig& config) {
    Position2D pos;
    pos.x = config.L1 * cos(theta1) + config.L2 * cos(theta1 + theta2);
    pos.y = config.L1 * sin(theta1) + config.L2 * sin(theta1 + theta2);
    return pos;
}

class DualArmJointPublisher : public rclcpp::Node {
public:
    DualArmJointPublisher() : Node("body_joint_publisher") {
        this->declare_parameter("arm.L1", 0.17);
        this->declare_parameter("arm.L2", 0.19);
        this->declare_parameter("arm.scale", 0.001); // este parámetro es para convertir de pixeles a metros, si mi imagen es de 640x480, eso esta mal porque deberia ser 640x480x0.001
        this->declare_parameter("arm.Kp", 0.1);
        this->declare_parameter("arm.error_threshold", 0.05);

        right_arm_config_.L1 = this->get_parameter("arm.L1").as_double();
        right_arm_config_.L2 = this->get_parameter("arm.L2").as_double();
        right_arm_config_.scale = this->get_parameter("arm.scale").as_double();
        right_arm_config_.Kp = this->get_parameter("arm.Kp").as_double();
        right_arm_config_.error_threshold = this->get_parameter("arm.error_threshold").as_double();
        left_arm_config_ = right_arm_config_;

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
            "body_tracker", 10, std::bind(&DualArmJointPublisher::arm_tracker_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DualArmJointPublisher::timer_callback, this));
    }

private:
    std::map<std::string, float> calculate_midpoints(const std::vector<std::string>& joints) {
        std::map<std::string, float> result;
        for (const auto& j : joints) {
            result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0;
        }
        return result;
    }

    std::map<std::string, float> process_arm_data(const std::array<float, 4>& angles, const std::vector<std::string>& arm_joints) {
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
        return positions;
    }

    void arm_tracker_callback(const buddy_interfaces::msg::BodyPosition::SharedPtr msg) {
        float tilt_angle = msg->shoulder_tilt_angle > 0 ? msg->shoulder_tilt_angle - 180 : msg->shoulder_tilt_angle + 180;
        float tilt_angle_radians = static_cast<float>(tilt_angle * M_PI / 180.0);
        torso_tilt_ = std::max(joint_limits_["joint_2"].first, std::min(joint_limits_["joint_2"].second, tilt_angle_radians));

        auto [right_angles, right_fk] = process_arm(msg, true);
        auto [left_angles, left_fk] = process_arm(msg, false);

        last_right_pos_ = right_angles;
        last_left_pos_ = left_angles;
    }

    std::pair<std::map<std::string, float>, Position2D> process_arm(const buddy_interfaces::msg::BodyPosition::SharedPtr msg, bool is_right_arm) {
        const std::vector<std::string> joints = is_right_arm ? 
            std::vector<std::string>{"joint_5", "joint_6", "joint_7", "joint_8"} :
            std::vector<std::string>{"joint_9", "joint_10", "joint_11", "joint_12"};

        const auto& config = is_right_arm ? right_arm_config_ : left_arm_config_;
        std::array<float, 4> angles = is_right_arm ? 
            std::array{msg->right_shoulder_elbow_zy, msg->right_shoulder_elbow_yx,
                       msg->right_elbow_wrist_yx, msg->right_elbow_wrist_zy} :
            std::array{msg->left_shoulder_elbow_zy, msg->left_shoulder_elbow_yx,
                       msg->left_elbow_wrist_yx, msg->left_elbow_wrist_zy};
        
        auto angles_map = process_arm_data(angles, joints);
        float theta1 = angles_map[joints[0]];
        float theta2 = angles_map[joints[1]];
        Position2D fk_pos = computeForwardKinematics(theta1, theta2, config);

        float mediapipe_x = (is_right_arm ? msg->right_wrist_x : msg->left_wrist_x) * config.scale;
        float mediapipe_y = (is_right_arm ? msg->right_wrist_y : msg->left_wrist_y) * config.scale;
        float error = sqrt(pow(fk_pos.x - mediapipe_x, 2) + pow(fk_pos.y - mediapipe_y, 2));
        
        if (error > config.error_threshold) {
            float desired_theta1 = atan2(mediapipe_y, mediapipe_x);
            float current_theta1 = atan2(fk_pos.y, fk_pos.x);
            angles_map[joints[0]] += config.Kp * (desired_theta1 - current_theta1);
            angles_map[joints[1]] += config.Kp * (error/(config.L1 + config.L2));

            for (const auto& joint : {joints[0], joints[1]}) {
                angles_map[joint] = std::max(joint_limits_[joint].first, 
                                           std::min(joint_limits_[joint].second, angles_map[joint]));
            }
        }

        return {angles_map, fk_pos};
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
    }

    std::map<std::string, std::pair<float, float>> joint_limits_;
    std::map<std::string, float> last_right_pos_;
    std::map<std::string, float> last_left_pos_;
    float torso_tilt_;
    ArmConfig right_arm_config_;
    ArmConfig left_arm_config_;
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