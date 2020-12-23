// Quick ROS example.
// The added CMakeLists make it easy to compile the package into ros libraries
// Just run catkin_make in a workspace with this code in it and then you can
// use the same Packetizer code on your PC to parse packets that you use on your
// Teensy to make them...

//#include <ros/ros.h>
#include <iostream>
//#include <std_msgs/String.h>
//#include <vector>

#include "Packetizer.h"

#define PACKETIZER_USE_CRC_AS_DEFAULT
bool USE_CRC = true;

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "rosnode_created_in_vs_code");
    //ros::NodeHandle n;


    Packetizer::Packet packet_in;
    for (int i = 0; i < 23; i++)
    {
        packet_in.data.push_back(i);
    }
    packet_in.index = 253;

    const auto& p_buff = Packetizer::encode(packet_in.data.data(), packet_in.data.size(), USE_CRC);
    const auto& packet_out = Packetizer::decode(p_buff.data.data(), p_buff.data.size(), USE_CRC);

        std::cout << "input   = "; for (const auto& p : packet_in.data) { std::cout << "[" << p << "]: " << (int)p << std::endl;  } 
        std::cout << "encoded = "; for (const auto& p : p_buff.data) {std::cout << "[" << p << "]: " << (int)p << std::endl;  } 
        std::cout << "decoded = "; for (const auto& p : packet_out.data) {std::cout << "[" << p << "]: " << (int)p << std::endl;  } 

        std::cout << "index_out = " << packet_out.index;

        if (packet_in.data != packet_out.data) {std::cout << "test 1 failed!";}
        else                         
            {std::cout << "test 1 success!";}

    //while (ros::ok())
    //{
    //    ros::spinOnce();
    //}
    return 0;
}