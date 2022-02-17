#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <memory>
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_interface");

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "move_group_interface/command", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      moveit::planning_interface::MoveGroupInterface move_group(*this, "moveit_arm_controller");
      planning_group = "moveit_arm_controller";
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      if (msg->data.c_str() == "Test"){
          this.moveToPose([0.0, 1.0, 0.25, 0.25]);
      }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string planning_group;

    // Set the move group interface to use this node to communicate with the planning group's controller

    const moveit::core::JointModelGroup* joint_model_group = 
        move_group_ptr->getCurrentState()->getJointModelGroup(planning_group);
    

    // Function to move the arm to a desired pose
    // position = [orientation.w, position.x, position.y, position.z]
    bool moveToPose(int position[])
    {
        // Set the target pose 
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = position[0];
        target_pose.position.x = position[1];
        target_pose.position.y = position[2];
        target_pose.position.z = position[3];
        move_group.setPoseTarget(target_pose);

        // Plan the arm trajectory
        moveit::planning_interface::MoveGroupInterface::Plan target_plan;
        bool success = (move_group.plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Log the result of planning the trajectory
        RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        // Move arm along plan
        move_group.move();

        return true;
    }



    int moveToJointPosition()
    {
        return 0;
    }

    // Function to move the arm to a desired pose along a cartesian path
    // position = [orientation.w, position.x, position.y, position.z]
    int moveToCartesianPath(int position[], moveit::planning_interface::MoveGroupInterface move_group)
    {

        geometry_msgs::msg::Pose target_pose = move_group.getCurrentState();

        return 0;
    }

    int handleArmCommand()
    {
        return 0;
    }
};



int main(int argc, char** argv)
{
    try
    {
        // Initalize move group interface node
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);

        //auto move_group_node = rclcpp::Node::make_shared('move_group_interface', node_options);
        MinimalSubscriber move_group_node = std::make_shared<MinimalSubscriber>());

        // Spin single threaded executor for the monitor to get information on the robot's state
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() {executor.spin(); }).detach();

        // Set the move group interface to use this node to communicate with the planning group's controller
        //static const std::string planning_group = 'moveit_arm_controller';
        //moveit::planning_interface::MoveGroupInterface move_group(move_group_node, planning_group);

        // Use pointers for improved performance
        //const moveit::core::JointModelGroup* joint_model_group = 
        //    move_group.getCurrentState()->getJointModelGroup(planning_group);

        // Set scaling factor for both velocity and acceleration in regard to the arm's movements
        //move_group.setMaxVelocityScalingFactor(0.1);
        //move_group.setMaxAccelerationScalingFactor(0.05);

        /*
        Add constraint to ensure end-effector stays flat

        Example:
            moveit_msgs::msg::OrientationConstraint ocm;
            ocm.link_name = "panda_link7";
            ocm.header.frame_id = "panda_link0";
            ocm.orientation.w = 1.0;
            ocm.absolute_x_axis_tolerance = 0.1;
            ocm.absolute_y_axis_tolerance = 0.1;
            ocm.absolute_z_axis_tolerance = 0.1;
            ocm.weight = 1.0;

            // Now, set it as the path constraint for the group.
            moveit_msgs::msg::Constraints test_constraints;
            test_constraints.orientation_constraints.push_back(ocm);
            move_group.setPathConstraints(test_constraints);

        */

        
        rclcpp::spin(move_group_node);
        rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
        std::cout << e.what();
    }
   
    
}