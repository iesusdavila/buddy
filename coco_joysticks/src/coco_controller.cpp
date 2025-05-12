#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class PS4RobotController : public rclcpp::Node
{
public:
  PS4RobotController()
  : Node("ps4_robot_controller")
  {
    // Define joint limits
    joint_names_ = {
      "joint_1", "joint_2", "joint_3", "joint_4", 
      "joint_5", "joint_6", "joint_7", "joint_8",
      "joint_9", "joint_10", "joint_11", "joint_12"
    };
    
    joint_min_limits_ = {
      -0.77, -0.3, -0.50, -0.25, 
      -0.75, 0.05, -0.6981, 0.15,
      -0.75, 0.05, -0.6981, 0.15
    };
    
    joint_max_limits_ = {
      0.77, 0.3, 0.5, 0.77, 
      1.50, 0.9848, 0.6981, 1.50,
      1.50, 0.9848, 0.6981, 1.50
    };
    
    current_positions_.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      current_positions_[i] = (joint_min_limits_[i] + joint_max_limits_[i]) / 2.0;
    }
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&PS4RobotController::joy_callback, this, std::placeholders::_1));
    
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/joint_trajectory_controller/follow_joint_trajectory");
    
    while (!action_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }
    
    RCLCPP_INFO(this->get_logger(), "PS4 Robot Controller started. Control scheme:");
    RCLCPP_INFO(this->get_logger(), "- D-pad Up/Down: Rotate hip (joint_1)");
    RCLCPP_INFO(this->get_logger(), "- D-pad Left/Right: Tilt torso (joint_2)");
    RCLCPP_INFO(this->get_logger(), "- Square/Circle: Rotate head (joint_3)");
    RCLCPP_INFO(this->get_logger(), "- Triangle/X: Tilt head (joint_4)");
    RCLCPP_INFO(this->get_logger(), "- R1 + sticks: Control right arm (joints 5-8)");
    RCLCPP_INFO(this->get_logger(), "- L1 + sticks: Control left arm (joints 9-12)");
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PS4RobotController::send_joint_command, this));
    
    should_send_command_ = false;
    
    movement_time_ = 0.5;
  }

private:
  enum PS4Buttons {
    CROSS = 0,       
    CIRCLE = 1,        
    TRIANGLE = 2,       
    SQUARE = 3,     
    L1 = 4,
    R1 = 5,
    L2_BTN = 6,
    R2_BTN = 7,
    SHARE = 8,
    OPTIONS = 9,
    PS = 10,
    L3 = 11,
    R3 = 12
  };
  
  enum PS4Axes {
    LEFT_STICK_X = 0,
    LEFT_STICK_Y = 1,
    L2_AXIS = 2,
    RIGHT_STICK_X = 3,
    RIGHT_STICK_Y = 4,
    R2_AXIS = 5,
    DPAD_X = 6,
    DPAD_Y = 7
  };
  
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    const double button_step = 0.05;
    const double speed_factor = 0.05;
    const double deadzone = 0.1;
    
    bool any_control_active = false;
    
    if (std::abs(joy_msg->axes[DPAD_Y]) > deadzone) {
      current_positions_[0] += joy_msg->axes[DPAD_Y] * button_step;
      any_control_active = true;
    }
    
    if (std::abs(joy_msg->axes[DPAD_X]) > deadzone) {
      current_positions_[1] += joy_msg->axes[DPAD_X] * button_step;
      any_control_active = true;
    }
    
    if (joy_msg->buttons[SQUARE]) {
      current_positions_[2] -= button_step;
      any_control_active = true;
    } else if (joy_msg->buttons[CIRCLE]) {
      current_positions_[2] += button_step;
      any_control_active = true;
    }
    
    if (joy_msg->buttons[TRIANGLE]) {
      current_positions_[3] += button_step;
      any_control_active = true;
    } else if (joy_msg->buttons[CROSS]) {
      current_positions_[3] -= button_step;
      any_control_active = true;
    }
    
    if (joy_msg->buttons[R1] && !joy_msg->buttons[L1]) {
      any_control_active = true;
      
      if (std::abs(joy_msg->axes[LEFT_STICK_Y]) > deadzone) {
        current_positions_[4] += joy_msg->axes[LEFT_STICK_Y] * speed_factor;
      }
      
      if (std::abs(joy_msg->axes[LEFT_STICK_X]) > deadzone) {
        current_positions_[5] += joy_msg->axes[LEFT_STICK_X] * speed_factor;
      }
      
      if (std::abs(joy_msg->axes[RIGHT_STICK_X]) > deadzone) {
        current_positions_[6] += joy_msg->axes[RIGHT_STICK_X] * speed_factor;
      }
      
      if (std::abs(joy_msg->axes[RIGHT_STICK_Y]) > deadzone) {
        current_positions_[7] += joy_msg->axes[RIGHT_STICK_Y] * speed_factor;
      }
    }
    
    if (joy_msg->buttons[L1] && !joy_msg->buttons[R1]) {
      any_control_active = true;
      
      if (std::abs(joy_msg->axes[LEFT_STICK_Y]) > deadzone) {
        current_positions_[8] += joy_msg->axes[LEFT_STICK_Y] * speed_factor;
      }
      
      if (std::abs(joy_msg->axes[LEFT_STICK_X]) > deadzone) {
        current_positions_[9] += joy_msg->axes[LEFT_STICK_X] * speed_factor;
      }
      
      if (std::abs(joy_msg->axes[RIGHT_STICK_X]) > deadzone) {
        current_positions_[10] += joy_msg->axes[RIGHT_STICK_X] * speed_factor;
      }
      
      if (std::abs(joy_msg->axes[RIGHT_STICK_Y]) > deadzone) {
        current_positions_[11] += joy_msg->axes[RIGHT_STICK_Y] * speed_factor;
      }
    }
    
    for (size_t i = 0; i < current_positions_.size(); ++i) {
      if (current_positions_[i] < joint_min_limits_[i]) {
        current_positions_[i] = joint_min_limits_[i];
      } else if (current_positions_[i] > joint_max_limits_[i]) {
        current_positions_[i] = joint_max_limits_[i];
      }
    }
    
    should_send_command_ = any_control_active;
  }
  
  void send_joint_command()
  {
    if (!should_send_command_) {
      return;
    }
    
    should_send_command_ = false;
    
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names_;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = current_positions_;
    point.velocities.resize(joint_names_.size(), 0.0);
    point.accelerations.resize(joint_names_.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(movement_time_);
    
    goal_msg.trajectory.points.push_back(point);
    
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&PS4RobotController::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&PS4RobotController::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&PS4RobotController::result_callback, this, std::placeholders::_1);
    
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
  
  void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Goal accepted by server");
    }
  }
  
  void feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
  {
    (void)feedback;
  }
  
  void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_DEBUG(this->get_logger(), "Goal succeeded");
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
  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::vector<std::string> joint_names_;
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;
  std::vector<double> current_positions_;
  
  bool should_send_command_;
  double movement_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PS4RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}