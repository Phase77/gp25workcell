/**
**  Simple ROS Node
**/
#include "gp25workcell/headers.h"

//Global Variables
geometry_msgs::Pose target_pose[20];
int Mcount = 0;
gp25workcell::MCommand Mmessage;
std::string PLANNING_GROUP = "manipulator";
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

// M Command callback message.
// This gets called whenever tcp_node recieves and M (move) command
// and publishes it on the "MCommandMsg" topic.
void MCommandMsgCallback(const gp25workcell::MCommand& MCommandmsg)
{
    Mmessage = MCommandmsg;
    Mcount++;
}

// Service function for "Target_Pose" service
// This service function is called when tcp_node revieves 
// a C (configure) command. 
bool SetTargetPose(gp25workcell::TargetPose::Request &req,
                   gp25workcell::TargetPose::Response &res)
{
  target_pose[req.poseNum].position.x = req.posX;
  target_pose[req.poseNum].position.y = req.posY;
  target_pose[req.poseNum].position.z = req.posZ;
  target_pose[req.poseNum].orientation.w = req.orientW; 
  target_pose[req.poseNum].orientation.x = req.orientX; 
  target_pose[req.poseNum].orientation.y = req.orientY; 
  target_pose[req.poseNum].orientation.z = req.orientZ; 
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Hello, World!");
  
  // Create a ROS node handle
  ros::NodeHandle nh_moveNode; 
 
  // subscribe to MasterMsg topic
  ros::Subscriber sub = node_handle.subscribe("MCommandMsg", 1000, MCommandMsgCallback);
  
  // Create and advertise "Target_Pose" service on ROS
  ros::ServiceServer service = node_handle.advertiseService("Target_Pose", SetTargetPose);
 
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      
  //Vector to scale 3D file units
  Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);

  // Define collision objects ROS message.
  moveit_msgs::CollisionObject collision_object;
  moveit_msgs::CollisionObject GoForm;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";
  GoForm.id = "GoForm";

  // Create mesh from stl file
  shapes::Mesh* m = shapes::createMeshFromResource("package://gp25workcell/GxSmall.stl", vectorScale);
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  //define pose of mesh
  geometry_msgs::Pose obj_pose;
  obj_pose.position.x = 0.0;
  obj_pose.position.y = 1.0;
  obj_pose.position.z = 1.0;
  obj_pose.orientation.w = sqrt(0.5);
  obj_pose.orientation.x = 0.0;
  obj_pose.orientation.y = 0.0;
  obj_pose.orientation.z = -sqrt(0.5);

  //Add mesh to collision object message
  GoForm.meshes.push_back(mesh);
  GoForm.mesh_poses.push_back(obj_pose);
  GoForm.operation = GoForm.ADD;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 1.0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 0;
  box_pose.position.x = 0.9;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  // collision_objects.push_back(GoForm);

  // // Now, let's add the collision object into the world
  // ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);


  // Define som initial poses
  //Pose for M1. Radians
  target_pose[1].position.x = 1.0;
  target_pose[1].position.y = 0.3;
  target_pose[1].position.z = 1.6;
  target_pose[1].orientation.w = 0.0;
  target_pose[1].orientation.x = 1.0;
  target_pose[1].orientation.y = 1.0;
  target_pose[1].orientation.z = 1.0;

  //Pose for M2. Radians
  target_pose[2].position.x = 1.0;
  target_pose[2].position.y = -1.0;
  target_pose[2].position.z = 1.1;
  target_pose[2].orientation.w = 0.0;
  target_pose[2].orientation.x = 1.0;
  target_pose[2].orientation.y = 1.0;
  target_pose[2].orientation.z = 1.0;

  //Pose for M3. Radians
  target_pose[3].position.x = 1.0;
  target_pose[3].position.y = 0.3;
  target_pose[3].position.z = 1.5;
  target_pose[3].orientation.w = 0.5;
  target_pose[3].orientation.x = 1.0;
  target_pose[3].orientation.y = 1.0;
  target_pose[3].orientation.z = 1.0;


  while(ros::ok())
  {
    if(Mcount >= 1)     // M command recieved
    {
      // set pose target to desired pose number
      move_group.setPoseTarget(target_pose[Mmessage.commandNum]);
      move_group.setMaxVelocityScalingFactor(0.1);

      // Plan move to target pose
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);    
      move_group.move();     
      Mcount--;
    }

    ros::spinOnce();
  }

  ros::shutdown();
  return 0;
}


  
