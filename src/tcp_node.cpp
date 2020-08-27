/**
 * Simple ROS Node
 **/
#include "gp25workcell/headers.h"
#include <sensor_msgs/JointState.h>


#define BUFFSIZE 1500

/**
 * ***************GLOBAL VARIABLES*************************
 */
sensor_msgs::JointState jointStateToPc;
ros::Publisher MCommandMsg_pub;             //Message to publish M commands on
gp25workcell::MCommand MCommandmsg;         //create message object to put data in and forward on MCommand topic
ros::ServiceClient client;

/**
 * ***************FUNCTIONS********************************
 */
void process_parsed_command();

// Sends all data in vbuf over TCP
void send_all(int sock, const void *vbuf, size_t size_buf)
{
    const double *buf = (double*)vbuf;
    int send_size;
    size_t size_left;
    const int flags = 0;

    size_left = size_buf;

    while(size_left > 0)
    {
        if((send_size = send(sock, buf, size_left, flags)) == -1)
        {
            ROS_INFO("send error");
            exit(1);
        }
        if(send_size == 0)
        {
            ROS_INFO("All bytes sent!");
        }

        size_left -= send_size;
        buf += send_size;
    }
    return;
}

// Callback function that listens on the "joint_state" topic
// Store the current joint state
void JointStateCallback(sensor_msgs::JointState msg)
{
    jointStateToPc = msg;
}

// Error message function
void Die(char *mess) { perror(mess); exit(1);}

int main(int argc, char* argv[])
{
    //this must be called before anything else ROS-related
    ros::init(argc, argv, "tcp_node");

    // Create a ROS node handle
    ros::NodeHandle nh_TcpNode;
    
    //Message to publish M commands on
    MCommandMsg_pub = nh_TcpNode.advertise<gp25workcell::MCommand>("MCommandMsg", 1000);

    // Create client for "Target_Pose" service
    client = nh_TcpNode.serviceClient<gp25workcell::TargetPose>("Target_Pose");
    
    ROS_INFO("Hello World TCP!!");

    //Subscribe to joint states
    ros::Subscriber jointSub = nh_TcpNode.subscribe("joint_states", 1000, JointStateCallback);
   
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
    
    // std::vector<double> pos;
    // pos = jointStateToPc.position;

    //Recieve data over TCP then forward data to TcpProcess_node
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
            msg.data = buffer;

            send_all(sock, (void *)jointStateToPc.position.data(), sizeof(double) * jointStateToPc.position.size());

            // Parse revieved command
            TCPparser.parse(buffer);

            // Process parsed command
            process_parsed_command();
        }

        ros::spinOnce();
    }

    fprintf(stdout, "\n");
    close(sock);


    //Dont exit the program
    ros::spin();
}

// Function used to process parsed command and act accordingly
void process_parsed_command()
{
    switch(TCPparser.command_letter)
    {
        case 'm':                                                   // M command move to preset point
        case 'M':
            // Publish desired move point for move_node to
            // recieve and plan move
            MCommandmsg.commandNum = TCPparser.codenum;
            MCommandMsg_pub.publish(MCommandmsg);                    
            break;
        case 'c':                                                   // C command configure target pose                 
        case 'C':
            // Load service request with data from parser
            // and call the service
            gp25workcell::TargetPose srv;
            srv.request.poseNum = TCPparser.codenum;
            srv.request.posX = TCPparser.param[0];
            srv.request.posY = TCPparser.param[1];
            srv.request.posZ = TCPparser.param[2];
            srv.request.orientW = TCPparser.param[3];
            srv.request.orientX = TCPparser.param[4];
            srv.request.orientY = TCPparser.param[5];
            srv.request.orientZ = TCPparser.param[6];

            if(client.call(srv))
            {
                ROS_INFO("The Response was true");
            }            
            else
            {
                ROS_INFO("The Response was false");
            }
            break;

    }
}