/**
 * Simple ROS Node
 **/
// #include "headers.h"
#include "gp25workcell/CmdParser.h"
#include <sensor_msgs/JointState.h>
#include "gp25workcell/MCommand.h"


#define BUFFSIZE 1500

/**
 * ***************FUNCTIONS********************************
 */
void process_parsed_command();

sensor_msgs::JointState jointStateToPc;
    //Message to publish M commands on
    ros::Publisher MCommandMsg_pub;
    //create message object to put data in and forward on MCommand topic
    gp25workcell::MCommand MCommandmsg;
    


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

void JointStateCallback(sensor_msgs::JointState msg)
{
    jointStateToPc = msg;
}


void Die(char *mess) { perror(mess); exit(1);}

int main(int argc, char* argv[])
{
    //this must be called before anything else ROS-related
    ros::init(argc, argv, "tcp_node");

    // Create a ROS node handle
    ros::NodeHandle nh_TcpNode;
    //Message to publish M commands on
    MCommandMsg_pub = nh_TcpNode.advertise<gp25workcell::MCommand>("MCommandMsg", 1000);




    ROS_INFO("Hello World TCP!!");

    //Subscribe to joint states
    ros::Subscriber jointSub = nh_TcpNode.subscribe("joint_states", 1000, JointStateCallback);

    ros::Publisher TcpMsg_pub = nh_TcpNode.advertise<std_msgs::String>("MasterMsg", 1000);
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
    
    std::vector<double> pos;
    pos = jointStateToPc.position;

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
            msg.data = buffer;//ss.str();

            // TcpMsg_pub.publish(msg);

            send_all(sock, (void *)jointStateToPc.position.data(), sizeof(double) * jointStateToPc.position.size());

            ROS_INFO("begin parse");
            TCPparser.parse(buffer);
            ROS_INFO("Parser Letter: %c", TCPparser.command_letter);
            ROS_INFO("Parser Codenum: %i", TCPparser.codenum);
            process_parsed_command();
        }

        ros::spinOnce();
    }

    fprintf(stdout, "\n");
    close(sock);


    //Dont exit the program
    ros::spin();
}

void process_parsed_command()
{
    switch(TCPparser.command_letter)
    {
        case 'm':
        case 'M':
            ROS_INFO("Process_parsed_command: Sending message");
            MCommandmsg.commandNum = TCPparser.codenum;
            MCommandMsg_pub.publish(MCommandmsg);                    
            break;

    }
}