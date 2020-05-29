/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <geometric_shapes/shape_operations.h>

int count = 0;
char message[20];

void MCommandMsgCallback(const std_msgs::String::ConstPtr& MCommandmsg)
{
    MCommandmsg->data.copy(message,10,0);
    count++;
    ROS_INFO("move_node: I heard: %s", MCommandmsg->data.c_str());
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Hello, World!");
  
  // subscribe to MasterMsg topic
  ros::Subscriber sub = node_handle.subscribe("MCommandMsg", 1000, MCommandMsgCallback);

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
 
  static const std::string PLANNING_GROUP = "manipulator";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  //Vector to scale 3D file units
  Eigen::Vector3d vectorScale(0.00254, 0.00254, 0.00254);

  // Define collision objects ROS message.
  moveit_msgs::CollisionObject collision_object;
  moveit_msgs::CollisionObject GoForm;

  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";
  // GoForm.id = "GoForm";

  // shapes::Mesh* m = shapes::createMeshFromResource("package://gp25workcell/GxSmall.stl", vectorScale);//,);

  // shape_msgs::Mesh mesh;
  // shapes::ShapeMsg mesh_msg;
  // shapes::constructMsgFromShape(m, mesh_msg);
  // mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  //define pose of mesh
  geometry_msgs::Pose obj_pose;
  obj_pose.position.x = 1.0;
  obj_pose.position.y = 1.0;
  obj_pose.position.z = 0.0;

  //Add mesh to collision object message
  // GoForm.meshes.push_back(mesh);
  // GoForm.mesh_poses.push_back(obj_pose);
  // GoForm.operation = GoForm.ADD;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 1.0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.9;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);
  // collision_objects.push_back(GoForm);

  // // Now, let's add the collision object into the world
  // ROS_INFO_NAMED("tutorial", "Add an object into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects);

  //Hard coded position values
  geometry_msgs::Pose target_pose_M1;
  geometry_msgs::Pose target_pose_M2;
  geometry_msgs::Pose target_pose_M3;

  //Joint positions for M1. Radians
  target_pose_M1.orientation.w = 0.0;
  target_pose_M1.orientation.x = 1.0;
  target_pose_M1.orientation.y = 1.0;
  target_pose_M1.orientation.z = 1.0;
  target_pose_M1.position.x = 1.0;
  target_pose_M1.position.y = 0.3;
  target_pose_M1.position.z = 1.6;

  //Joint positions for M2. Radians
  target_pose_M2.orientation.w = 1.0;
  target_pose_M2.orientation.x = 1.0;
  target_pose_M2.orientation.y = 1.0;
  target_pose_M2.orientation.z = 1.0;
  target_pose_M2.position.x = 1.0;
  target_pose_M2.position.y = 0.3;
  target_pose_M2.position.z = 1.1;

  //Joint positions for M3. Radians
  target_pose_M3.orientation.w = 0.5;
  target_pose_M3.orientation.x = 1.0;
  target_pose_M3.orientation.y = 1.0;
  target_pose_M3.orientation.z = 1.0;
  target_pose_M3.position.x = 1.0;
  target_pose_M3.position.y = 0.3;
  target_pose_M3.position.z = 1.5;

  while(ros::ok())
  {
    if(count >= 1)
    {
      if(message[0] = 'M')
      {
        switch(message[1])
        {
          case '1':
            //if recieved M1
            ROS_INFO("move_node: Move to M1");
            move_group.setPoseTarget(target_pose_M1);
            break;
          case '2':
            //if recieved M1
            ROS_INFO("move_node: Move to M2");
            move_group.setPoseTarget(target_pose_M2);
            break; 
          case '3':
            //if recieved M1
            ROS_INFO("move_node: Move to M3");
            move_group.setPoseTarget(target_pose_M3);
            break; 
        }
        move_group.setMaxVelocityScalingFactor(0.1);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);    
        move_group.move();
      }
      count--;
    }
    ros::spinOnce();
  }



  // // Next get the current set of joint values for the group.
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  // joint_group_positions[0] = -1.0;  // radians
  // move_group.setJointValueTarget(joint_group_positions);
  // move_group.setMaxVelocityScalingFactor(0.1);

  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // move_group.move();

  


  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
