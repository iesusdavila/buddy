#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "coco_interfaces/msg/body_position.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <map>
#include <string>
#include <vector>

class DualArmTrajectoryController : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    DualArmTrajectoryController() : Node("body_trajectory_controller") {
        // Define joint limits
        joint_limits_["joint_2"] = std::make_pair(-0.3, 0.3);  
        
        joint_limits_["joint_5"] = std::make_pair(-0.7853, 1.5708);
        joint_limits_["joint_6"] = std::make_pair(0.0, 1.0472);
        joint_limits_["joint_7"] = std::make_pair(-0.7853, 0.7853);
        joint_limits_["joint_8"] = std::make_pair(0.1745, 1.5708);
        
        joint_limits_["joint_9"] = std::make_pair(-0.7853, 1.5708);
        joint_limits_["joint_10"] = std::make_pair(0.0, 1.0472);
        joint_limits_["joint_11"] = std::make_pair(-0.7853, 0.7853);
        joint_limits_["joint_12"] = std::make_pair(0.1745, 1.5708);
        
        right_joints_ = {"joint_5", "joint_6", "joint_7", "joint_8"};
        left_joints_ = {"joint_9", "joint_10", "joint_11", "joint_12"};
        
        // Initialize joint positions to midpoints
        last_right_pos_ = calculateMidpoints(right_joints_);
        last_left_pos_ = calculateMidpoints(left_joints_);
        
        torso_tilt_ = 0.0;
        
        // Action client for the joint trajectory controller
        trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/joint_trajectory_controller/follow_joint_trajectory");
            
        // Wait for the action server to be available
        if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting 5 seconds");
            throw std::runtime_error("Action server not available");
        }
        
        // Subscribe to body tracking data
        subscription_ = this->create_subscription<coco_interfaces::msg::BodyPosition>(
            "body_tracker", 10, 
            std::bind(&DualArmTrajectoryController::armTrackerCallback, this, std::placeholders::_1));
        
        // Timer to send trajectory goals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250), 
            std::bind(&DualArmTrajectoryController::sendTrajectoryGoal, this));
        
        // Initialize the joint names used in the trajectory
        all_joints_ = {
            "joint_1", "joint_2", "joint_3", "joint_4",
            "joint_5", "joint_6", "joint_7", "joint_8",
            "joint_9", "joint_10", "joint_11", "joint_12"
        };
        
        new_data_available_ = false;
        goal_sent_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Dual arm trajectory controller initialized");
    }

private:
    const float MAX_CHANGE_PER_STEP = 0.5;

    std::map<std::string, float> calculateMidpoints(const std::vector<std::string>& joints) {
        std::map<std::string, float> result;
        for (const auto& j : joints) {
            result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0;
        }
        return result;
    }

    float euler2Radian(float euler) {
        return euler * M_PI / 180.0;
    }

    float limitJointPosition(const std::string& joint, float position) {
        if (joint_limits_.find(joint) != joint_limits_.end()) {
            float lower = joint_limits_[joint].first;
            float upper = joint_limits_[joint].second;
            return std::max(lower, std::min(upper, position));
        }
        return position;
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
    
    void armTrackerCallback(const coco_interfaces::msg::BodyPosition::SharedPtr msg) {
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
        
        last_right_pos_ = processArmData(right_angles, right_joints_, true);
        
        std::array<float, 4> left_angles = {
            msg->left_shoulder_elbow_zy,
            msg->left_shoulder_elbow_yx,
            msg->left_elbow_wrist_yx,
            msg->left_elbow_wrist_zy
        };
        
        last_left_pos_ = processArmData(left_angles, left_joints_, false);
        
        new_data_available_ = true;
    }
    
    // Callback function for trajectory goal response
    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            goal_sent_ = true;
        }
    }
    
    // Callback function for trajectory feedback
    void feedback_callback(
        GoalHandleFollowJointTrajectory::SharedPtr,
        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
    
    // Callback function for trajectory result
    void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        goal_sent_ = false;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
    
    // Method to send trajectory goal
    void sendTrajectoryGoal() {
        // Only send a new goal if we have new data and no goal is currently being processed
        if (!new_data_available_ || goal_sent_) {
            return;
        }
        
        auto goal_msg = FollowJointTrajectory::Goal();
        
        // Create the joint trajectory
        goal_msg.trajectory.joint_names = all_joints_;
        
        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {
            0.0,                   // joint_1
            torso_tilt_,           // joint_2
            0.0, 0.0,              // joint_3, joint_4
            last_right_pos_["joint_5"],
            last_right_pos_["joint_6"],
            last_right_pos_["joint_7"],
            0.87265,               // joint_8
            last_left_pos_["joint_9"],
            last_left_pos_["joint_10"],
            last_left_pos_["joint_11"],
            0.87265                // joint_12
        };
        
        // Set velocities to zero for smoother movement
        point.velocities.resize(point.positions.size(), 0.0);
        
        // Set trajectory time from start
        point.time_from_start = rclcpp::Duration::from_seconds(0.5);
        
        // Add the point to the trajectory
        goal_msg.trajectory.points.push_back(point);
        
        // Set goal time tolerance
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);
        
        // Send the goal
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&DualArmTrajectoryController::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&DualArmTrajectoryController::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&DualArmTrajectoryController::result_callback, this, std::placeholders::_1);
            
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        trajectory_client_->async_send_goal(goal_msg, send_goal_options);
        
        new_data_available_ = false;
    }
    
    // Joint limits and positions
    std::map<std::string, std::pair<float, float>> joint_limits_;
    std::map<std::string, float> last_right_pos_;
    std::map<std::string, float> last_left_pos_;
    float torso_tilt_;

    std::vector<std::string> right_joints_;
    std::vector<std::string> left_joints_;

    std::vector<std::string> all_joints_;
    
    // Action client
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
    
    // Subscription and timer
    rclcpp::Subscription<coco_interfaces::msg::BodyPosition>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Control flags
    bool new_data_available_;
    bool goal_sent_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmTrajectoryController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}