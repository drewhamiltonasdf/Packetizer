// Quick ROS example.
// The added CMakeLists make it easy to compile the package into ros libraries
// Just run catkin_make in a workspace with this code in it and then you can
// use the same Packetizer code on your PC to parse packets that you use on your
// Teensy to make them...

//#include <ros/ros.h>
#include <iostream>
#include <iomanip>              //For std::setprecision
#include <string.h>             //For strcpy (if you need to set up char[] fields)
//#include <std_msgs/String.h>
//#include <vector>

#include "Packetizer.h"
#include "type_packer.h"

// You need to define your platform the same as is defined
// on your microcontroller. As far as I can tell, this mostly
// has to do with selecting which type of CRC to use, and some
// business with STL containers vs. hideataki's custom containers
// I was using Teensy, so I added my
// own define, but you could also #define Arduino, or use the
// default #define for Teensy 3.x #define KINETISK
#define ROS_TEENSY_CRC

#define PACKETIZER_USE_CRC_AS_DEFAULT
#define PACKETIZER_USE_INDEX_AS_DEFAULT

bool USE_CRC = true;
bool USE_INDEX = true;

#define JOINT_POSITION_PACKET   123         // Using index as packet-type declaration
#define REQUEST_STATUS_PACKET   239         //

#define UNUSED_FLAG0            0           // Bit flags, we use these for our 8-bit flags bool
#define UNUSED_FLAG1            1
#define UNUSED_FLAG2            2
#define UNUSED_FLAG3            3
#define COMPLETED_FLAG          4
#define WAITING_FLAG            5
#define BUSY_FLAG               6
#define ERROR_FLAG              7

#define NUM_JOINTS              7

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "rosnode_created_in_vs_code");
    //ros::NodeHandle n;

    Packetizer::Packet packet_in;
    packet_in.index = 123;

    uint8_t MCU_flags; 
        setFlag(MCU_flags, UNUSED_FLAG0,    0);
        setFlag(MCU_flags, UNUSED_FLAG1,    0);
        setFlag(MCU_flags, UNUSED_FLAG2,    0);
        setFlag(MCU_flags, UNUSED_FLAG3,    0);
        setFlag(MCU_flags, COMPLETED_FLAG,  0);
        setFlag(MCU_flags, WAITING_FLAG,    0);
        setFlag(MCU_flags, BUSY_FLAG,       1);
        setFlag(MCU_flags, ERROR_FLAG,      0);
    
    std::vector<float> JOINT_POSITIONS{0, -3.1415, 0, 6.2, 0, 99, 0};

    push_back_type(packet_in.data, MCU_flags, JOINT_POSITIONS);

    uint8_t receive_flags;
    std::vector<float> receive_JOINTS(NUM_JOINTS);

    const auto& p_buff = Packetizer::encode(packet_in.index, packet_in.data.data(), packet_in.data.size(), USE_CRC);
    const auto& packet_out = Packetizer::decode(p_buff.data.data(), p_buff.data.size(), USE_INDEX, USE_CRC);

    unpack_type(packet_out.data, receive_flags, receive_JOINTS);

    std::cout << std::endl << "MCU Flags: ";
    printFlags(receive_flags);

    std::cout << std::endl << "Joint Positions after unpacking: " << std::endl;
    int i = 0;
    for (auto joint_pos : receive_JOINTS)
    {
        std::cout << "  [" << i++ << "]: " << std::setprecision(5) << joint_pos << std::endl;
    }
    std::cout << std::endl;

    //while (ros::ok())
    //{
    //    ros::spinOnce();
    //}
    return 0;
}