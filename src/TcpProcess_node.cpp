/**
 * Simple ROS Node
 **/
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

int count = 0;
char message[20];

void MasterMsgCallback(const std_msgs::String::ConstPtr& msg)
{
    msg->data.copy(message,10,0);
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
    
    //Message to publish M commands on
    ros::Publisher MCommandMsg_pub = nh_TcpProcess.advertise<std_msgs::String>("MCommandMsg", 1000);
    
    //create message object to put data in and forward on MCommand topic
    std_msgs::String MCommandmsg;

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
                    ROS_INFO("TcpProcess_node: I got a Report command");
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