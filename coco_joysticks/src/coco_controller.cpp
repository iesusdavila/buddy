#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <cmath>

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

    current_positions_ = {
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.3,
      0.0, 0.1, 0.0, 0.3
    };
    
    joystick_offsets_.resize(8, 0.0);
    calibration_samples_.resize(8);
    calibration_count_ = 0;
    is_calibrated_ = false;
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      current_positions_[i] = std::min(
          std::max(current_positions_[i], joint_min_limits_[i]), 
          joint_max_limits_[i]
      );
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
    
    RCLCPP_INFO(this->get_logger(), "PS4 Robot Controller started.");
    RCLCPP_INFO(this->get_logger(), "Calibrating joysticks... Please don't touch the controller for 3 seconds.");
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PS4RobotController::send_joint_command, this));
    
    should_send_command_ = false;
    movement_time_ = 1.0;
  }

private:
  enum PS4Buttons {
    CROSS = 1, CIRCLE = 2, TRIANGLE = 3, SQUARE = 0,
    L1 = 4, R1 = 5, L2_BTN = 6, R2_BTN = 7,
    SHARE = 8, OPTIONS = 9, PS = 10, L3 = 11, R3 = 12
  };
  
  enum PS4Axes {
    LEFT_STICK_X = 0, LEFT_STICK_Y = 1, L2_AXIS = 3,
    RIGHT_STICK_X = 2, RIGHT_STICK_Y = 5, R2_AXIS = 4,
    DPAD_X = 6, DPAD_Y = 7
  };
  
  double apply_deadzone(double value, int axis_index, double deadzone = 0.1) {
    double corrected_value = value - joystick_offsets_[axis_index];
    
    if (std::abs(corrected_value) < deadzone) {
      return 0.0;
    }
    
    double sign = (corrected_value > 0) ? 1.0 : -1.0;
    double normalized = (std::abs(corrected_value) - deadzone) / (1.0 - deadzone);
    return sign * std::min(normalized, 1.0);
  }
  
  void calibrate_joysticks(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    if (calibration_count_ < 30) {
      for (int i = 0; i < 8; ++i) {
        if (i < static_cast<int>(joy_msg->axes.size())) {
          calibration_samples_[i].push_back(joy_msg->axes[i]);
        }
      }
      calibration_count_++;
      return;
    }
    
    if (!is_calibrated_) {
      for (int i = 0; i < 8; ++i) {
        if (!calibration_samples_[i].empty()) {
          double sum = 0.0;
          for (double sample : calibration_samples_[i]) {
            sum += sample;
          }
          joystick_offsets_[i] = sum / calibration_samples_[i].size();
        }
      }
      
      is_calibrated_ = true;
      RCLCPP_INFO(this->get_logger(), "Joystick calibration completed!");
      RCLCPP_INFO(this->get_logger(), "Offsets - LX: %.3f, LY: %.3f, RX: %.3f, RY: %.3f", 
                  joystick_offsets_[LEFT_STICK_X], joystick_offsets_[LEFT_STICK_Y],
                  joystick_offsets_[RIGHT_STICK_X], joystick_offsets_[RIGHT_STICK_Y]);
      
      calibration_samples_.clear();
    }
  }
  
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (!is_calibrated_) {
      calibrate_joysticks(joy_msg);
      return;
    }
    
    const double button_step = 0.05;
    const double speed_factor = 0.05;
    const double deadzone = 0.15; 
    
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
    
    if (joy_msg->buttons[CROSS]) {
      current_positions_[3] += button_step;
      any_control_active = true;
    } else if (joy_msg->buttons[TRIANGLE]) {
      current_positions_[3] -= button_step;
      any_control_active = true;
    }
    
    if (joy_msg->buttons[L1] && !joy_msg->buttons[R1]) {   
      double left_y = apply_deadzone(joy_msg->axes[LEFT_STICK_Y], LEFT_STICK_Y, deadzone);
      double left_x = apply_deadzone(joy_msg->axes[LEFT_STICK_X], LEFT_STICK_X, deadzone);
      double right_x = apply_deadzone(joy_msg->axes[RIGHT_STICK_X], RIGHT_STICK_X, 0.01);
      double right_y = apply_deadzone(joy_msg->axes[RIGHT_STICK_Y], RIGHT_STICK_Y, 0.01);
      
      if (std::abs(left_y) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "L1 + Left Stick Y: %.3f", left_y);
        any_control_active = true;
        current_positions_[4] += left_y * speed_factor;
      }
      
      if (std::abs(left_x) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "L1 + Left Stick X: %.3f", left_x);
        any_control_active = true;
        current_positions_[5] += left_x * speed_factor;
      }
      
      if (std::abs(right_x) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "L1 + Right Stick X: %.3f", right_x);
        any_control_active = true;
        current_positions_[6] += right_x * speed_factor;
      }
      
      if (std::abs(right_y) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "L1 + Right Stick Y: %.3f", right_y);
        any_control_active = true;
        current_positions_[7] += right_y * speed_factor;
      }
    }
    
    if (joy_msg->buttons[R1] && !joy_msg->buttons[L1]) {
      double left_y = apply_deadzone(joy_msg->axes[LEFT_STICK_Y], LEFT_STICK_Y, deadzone);
      double left_x = apply_deadzone(joy_msg->axes[LEFT_STICK_X], LEFT_STICK_X, deadzone);
      double right_x = apply_deadzone(joy_msg->axes[RIGHT_STICK_X], RIGHT_STICK_X, deadzone);
      double right_y = apply_deadzone(joy_msg->axes[RIGHT_STICK_Y], RIGHT_STICK_Y, deadzone);
      
      if (std::abs(left_y) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "R1 + Left Stick Y: %.3f", left_y);
        any_control_active = true;
        current_positions_[8] += left_y * speed_factor;
      }
      
      if (std::abs(left_x) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "R1 + Left Stick X: %.3f", left_x);
        any_control_active = true;
        current_positions_[9] += -left_x * speed_factor;
      }
      
      if (std::abs(right_x) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "R1 + Right Stick X: %.3f", right_x);
        any_control_active = true;
        current_positions_[10] += right_x * speed_factor;
      }
      
      if (std::abs(right_y) > 0.0) {
        RCLCPP_INFO(this->get_logger(), "R1 + Right Stick Y: %.3f", right_y);
        any_control_active = true;
        current_positions_[11] += right_y * speed_factor;
      }
    }

    if (joy_msg->buttons[R1] && joy_msg->buttons[L1]) {
      double left_y = apply_deadzone(joy_msg->axes[LEFT_STICK_Y], LEFT_STICK_Y, deadzone);
      double left_x = apply_deadzone(joy_msg->axes[LEFT_STICK_X], LEFT_STICK_X, deadzone);
      double right_x = apply_deadzone(joy_msg->axes[RIGHT_STICK_X], RIGHT_STICK_X, deadzone);
      double right_y = apply_deadzone(joy_msg->axes[RIGHT_STICK_Y], RIGHT_STICK_Y, deadzone);
      
      if (std::abs(left_y) > 0.0) {
        any_control_active = true;
        current_positions_[4] += left_y * speed_factor;
        current_positions_[8] += left_y * speed_factor;
      }
      
      if (std::abs(left_x) > 0.0) {
        any_control_active = true;
        current_positions_[5] += left_x * speed_factor;
        current_positions_[9] += -left_x * speed_factor;
      }
      
      if (std::abs(right_x) > 0.0) {
        any_control_active = true;
        current_positions_[6] += right_x * speed_factor;
        current_positions_[10] += right_x * speed_factor;
      }
      
      if (std::abs(right_y) > 0.0) {
        any_control_active = true;
        current_positions_[7] += right_y * speed_factor;
        current_positions_[11] += right_y * speed_factor;
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
    
    RCLCPP_INFO(this->get_logger(), "Publicando angulo");
    
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
  
  std::vector<double> joystick_offsets_;
  std::vector<std::vector<double>> calibration_samples_;
  int calibration_count_;
  bool is_calibrated_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PS4RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}