#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


// Function to create a target pose for the robot's end effector 
geometry_msgs::msg::PoseStamped create_target_pose(const std::shared_ptr<rclcpp::Node> &node, 
  float x, float y, float z, 
  float qx, float qy, float qz, float qw) 
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.header.stamp = node->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.x = qx;
  pose.pose.orientation.y = qy;
  pose.pose.orientation.z = qz;
  pose.pose.orientation.w = qw;
  return pose;
}

// Function to plan and execute a path to a target pose
// This function takes a target pose, a MoveGroupInterface object, and a logger as input
// It sets the target pose for the robot's end effector, plans a path to that pose,
// and executes the planned path
// If the planning is successful, it prints a success message and executes the plan
// If the planning fails, it prints an error message
// The function returns nothing
void planAndExecute(const geometry_msgs::msg::PoseStamped &target_pose, 
  moveit::planning_interface::MoveGroupInterface::Plan &plan, 
  moveit::planning_interface::MoveGroupInterface &move_group, 
  const rclcpp::Logger &logger) 
{
  // Set the target pose for the robot's end effector
  move_group.setPoseTarget(target_pose);

  // Plan a path to the target pose
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Print the result of the planning
  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful!");
    // Execute the planned path
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
}

int main(int argc, char ** argv)
{
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Create a node for the robot called "myrobot_node". 
  //auto const node = rclcpp::Node::make_shared("myrobot_node");
  // If I want handle or change any settings, I can do it  with the follow line
  auto const node = rclcpp::Node::make_shared("myrobot_node", rclcpp::NodeOptions().use_intra_process_comms(true));

  // Create a logger to print messages to the console
  auto logger = rclcpp::get_logger("myrobot_node");
  // Print a message to the console
  RCLCPP_INFO(logger, "Starting myrobot_node...");

  // Create the MoveIt! MoveGroupInterface
  // This interface allows us to control the robot's arm and plan movements
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "me6_group");

  // Specify the planning pipeline to use
  move_group->setPlanningPipelineId("ompl");
  // Specify the planner to use
  move_group->setPlannerId("RRTConnectkConfigDefault");
  // Specify the maximum amount of time in time to plan a path (in seconds)
  move_group->setPlanningTime(5.0);
  // Set the scaling factor for the velocity and acceleration of the robot's movements
  move_group->setMaxVelocityScalingFactor(1.0);
  move_group->setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "MoveGroupInterface initialized.");
  RCLCPP_INFO(logger, "Robot name: %s", move_group->getName().c_str());
  RCLCPP_INFO(logger, "Planning Pipeline: %s", move_group->getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", move_group->getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %f seconds", move_group->getPlanningTime());
  
  // Create a target poses for the robot's end effector
  auto target_pose_1 = create_target_pose(node, -0.3, 0.0, 0.20, -0.707, 0.707, 0.0, 0.0);
  auto target_pose_2 = create_target_pose(node, -0.133, 0.0, 0.61686, -0.5, 0.5, 0.5, -0.5);
  
  // Plan a path to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  // Plan and execute a path to the target pose
  planAndExecute(target_pose_1, plan, *move_group, logger);
 
  // Sleep for 10 seconds to allow the user to see the final pose
  rclcpp::sleep_for(std::chrono::seconds(5));
  
  // Plan and execute a path to the target pose
  planAndExecute(target_pose_2, plan, *move_group, logger);

  // Shutdown ROS 2  
  rclcpp::shutdown();
  // Return success
  RCLCPP_INFO(logger, "myrobot_node finished.");
  return 0;
}
