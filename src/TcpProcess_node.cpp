/**
 * Simple ROS Node
 **/
#include "headers.h"


int count = 0;
char message[88];

void MasterMsgCallback(const std_msgs::String::ConstPtr& msg)
{
    msg->data.copy(message,44,0);
    count++;
    ROS_INFO("I heard: %s", msg->data.c_str());
}

int main(int argc, char* argv[])
{
    //this must be called before anything else ROS-related
    ros::init(argc, argv, "TcpProcess_node");

    // Create a ROS node handle
    ros::NodeHandle nh_TcpProcess;

    ROS_INFO("Hello World TcpProcess!!");

    //subscribe to MasterMsg topic
    ros::Subscriber sub = nh_TcpProcess.subscribe("MasterMsg", 1000, MasterMsgCallback);

    //subscribe to joint_states topic
    // ros::Subscriber jointSub = nh_TcpProcess.subscribe("joint_states", 1000, JointStateCallback);
    
    //Message to publish M commands on
    ros::Publisher MCommandMsg_pub = nh_TcpProcess.advertise<std_msgs::String>("MCommandMsg", 1000);
    
    //create message object to put data in and forward on MCommand topic
    std_msgs::String MCommandmsg;

    //Message to publish C commands on
    ros::Publisher CCommandMsg_pub = nh_TcpProcess.advertise<std_msgs::String>("CCommandMsg", 10000);
    
    //create message object to put data in and forward on CCommand topic
    std_msgs::String CCommandmsg;

    //Message to publish R commands on
    ros::Publisher RCommandMsg_pub = nh_TcpProcess.advertise<std_msgs::String>("RCommandMsg", 10000);
    
    //create message object to put data in and forward on RCommand topic
    std_msgs::String RCommandmsg;

    // // The :move_group_interface:`MoveGroup` class can be easily
    // // setup using just the name of the planning group you would like to control and plan for.
    // moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    while(ros::ok())
    {
        if(count >= 1)
        {
            //if M command(move)
            ROS_INFO("%s", message);
            ROS_INFO("%c", message[0]);
            ROS_INFO("%c", message[1]);
            switch(message[0])
            {
                case 'M':
                    ROS_INFO("TcpProcess_node: I got a move command");
                    switch(message[1])
                    {
                        case '1':
                            ROS_INFO("TcpProcess_node: Move to M1");
                            MCommandmsg.data = "M1";
                            break;
                        case '2':
                            ROS_INFO("TcpProcess_node: Move to M2");
                            MCommandmsg.data = "M2";
                            break;
                        case '3':
                            ROS_INFO("TcpProcess_node: Move to M3");
                            MCommandmsg.data = "M3";
                            break;
                        default:
                            ROS_INFO("TcpProcess_node: Unknown move command");
                            MCommandmsg.data = "Unknown Move Command";
                            break;
                    }
                    MCommandMsg_pub.publish(MCommandmsg);                    
                    break;
                case 'R':
                    ROS_INFO("TcpProcess_node: I got a Report Command");
                    switch(message[1])
                    {
                        case '1':
                            ROS_INFO("TcpProcess_node: Report R1");
                            RCommandmsg.data = "R1";
                            break;
                        case '2':
                            ROS_INFO("TcpProcess_node: Report R2");
                            RCommandmsg.data = "R2";
                            break;
                        case '3':
                            ROS_INFO("TcpProcess_node: Report R3");
                            RCommandmsg.data = "R3";
                            break;
                        default:
                            ROS_INFO("TcpProcess_node: Unknown report command");
                            RCommandmsg.data = "Unknown Report Command";
                            break;
                    }
                    RCommandMsg_pub.publish(RCommandmsg);
                    break;
                case 'C':
                    ROS_INFO("TcpProcess_node: I got a Config command");
                    CCommandmsg.data = message;
                    CCommandMsg_pub.publish(CCommandmsg);
                    break;
                default:
                    ROS_INFO("TcpProcess_node: ???????"); 
                    break;
            }
            count--;
        }
        ros::spinOnce();
    }
    
    //Dont exit the program
    ros::spin();
}