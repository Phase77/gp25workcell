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
#include <fcntl.h>

#define BUFFSIZE 1500

void Die(char *mess) { perror(mess); exit(1);}

int main(int argc, char* argv[])
{
    //this must be called before anything else ROS-related
    ros::init(argc, argv, "tcp_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    ROS_INFO("Hello World TCP!!");

    ros::Publisher TcpMsg_pub = nh.advertise<std_msgs::String>("MasterMsg", 1000);
    int count = 0;

    //Create socket
    int sock;
    struct sockaddr_in echoserver;
    char buffer[BUFFSIZE];
    unsigned int echolen;
    int received = 0;

    if((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    {
        Die("Failed to create socket");
    }


    //Construct the server sockaddr_in structure
    memset(&echoserver, 0, sizeof(echoserver));
    echoserver.sin_family = AF_INET;
    echoserver.sin_addr.s_addr = inet_addr(argv[1]);
    echoserver.sin_port = htons(atoi(argv[3]));
    //Establish connection
    if(connect(sock,
                (struct sockaddr *) &echoserver,
                sizeof(echoserver)) < 0)
    {
        Die("Failed to connect with server");
    }

    //Add recieve timeout
    fcntl(sock, F_SETFL, O_NONBLOCK);

    //Recieve data onver TCP then forward data to TcpProcess_node
    // on MasterMsg topic
    while(ros::ok())
    {
        int bytes = 0;
        if((bytes = recv(sock, buffer, BUFFSIZE-1, 0)) >= 1)
        {
            received += bytes;
            buffer[bytes] = '\0';
            fprintf(stdout, buffer);
            send(sock, "got message", strlen("got message"), 0);

            //create message object to put data in and forward on MasterMsg topic
            std_msgs::String msg;

            // std::stringstream ss;
            // ss<< "Hello Topic " << count;
            msg.data = buffer;//ss.str();

            TcpMsg_pub.publish(msg);

            ROS_INFO("%s", msg.data.c_str());
        }

        ros::spinOnce();
    }

    fprintf(stdout, "\n");
    close(sock);


    //Dont exit the program
    ros::spin();
}