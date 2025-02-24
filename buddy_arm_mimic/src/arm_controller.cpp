#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <buddy_interfaces/msg/hand_position.hpp>

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node {
public:
    ArmController() : Node("arm_controller") {
        // Inicializar MoveGroupInterface primero
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](auto*){}),  // Fix ownership
            "right_arm");
            
        // Configurar parámetros de movimiento
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPlanningTime(1.5);

        // Crear subscription después de inicializar MoveIt
        subscription_ = this->create_subscription<buddy_interfaces::msg::HandPosition>(
            "/hand_position", 10,
            [this](const buddy_interfaces::msg::HandPosition::SharedPtr msg) {
                this->callback(msg);
            });
    }

private:
    void callback(const buddy_interfaces::msg::HandPosition::SharedPtr msg) {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = msg->x;
        target_pose.position.y = msg->y;
        target_pose.position.z = msg->z;
        target_pose.orientation.w = 1.0;

        move_group_->setPoseTarget(target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan)) {
            move_group_->execute(plan);
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<buddy_interfaces::msg::HandPosition>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}