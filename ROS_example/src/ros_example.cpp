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

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "rosnode_created_in_vs_code");
    //ros::NodeHandle n;


    Packetizer::Packet packet_in;
    
    //Add in a non-char sized thing to our packet using this function defined in type_packer.h
    uint8_t asdf1 = 'A'; push_back_type(packet_in.data, asdf1);
    uint8_t asdf2 = 'B'; push_back_type(packet_in.data, asdf2);
    uint8_t asdf3 = 'C'; push_back_type(packet_in.data, asdf3);
    
    // This will cause an error, we can't have a variadic packet with multiple types
    // float asdf4 =   17.934; push_back_type(packet_in.data, asdf4);    
    
    packet_in.index = 123;

    const auto& p_buff = Packetizer::encode(packet_in.index, packet_in.data.data(), packet_in.data.size(), USE_CRC);
    
    //inline const Packet& decode(const uint8_t* data, const size_t size, const bool b_index, const bool b_crc)
    const auto& packet_out = Packetizer::decode(p_buff.data.data(), p_buff.data.size(), USE_INDEX, USE_CRC);

        std::cout << "input   = " << std::endl << std::endl; for (const auto& p : packet_in.data) { std::cout << "[" << p << "]: " << (int)p << std::endl;  }  std::cout << std::endl;
        std::cout << "encoded = " << std::endl << std::endl; for (const auto& p : p_buff.data) {std::cout << "[" << p << "]: " << (int)p << std::endl;  }  std::cout << std::endl;
        std::cout << "decoded = " << std::endl << std::endl; for (const auto& p : packet_out.data) {std::cout << "[" << p << "]: " << (int)p << std::endl;  } std::cout << std::endl;

        std::cout << "index_out = [" << (int)packet_out.index << "]" << std::endl;

        if (packet_in.data != packet_out.data) {std::cout << "test 1 failed!"<< std::endl;}
        else                         
            {std::cout << std::endl << "test 1 success!"<< std::endl;}

    std::cout << "CRC is expected to be = " << (int)crcx::crc8(packet_in.data.data(), packet_in.data.size()) << std::endl;
    std::cout << "CRC from encoded packet is = " << (int)p_buff.data.at(static_cast<int>(p_buff.data.size()) - 2) << std::endl;
    //std::cout << "CRC = " << (int)FastCRC::crc8(packet_out.data.data(), p_buff.data.size());

    // Here we unpack our packet into doubles. But you can change the template type
    // and get any kind of type you want as long as that's what you made the packet out
    // of in the first place. In other words, if you have a datatype that is 4 bytes long
    // your packet ought to be divisible by 4. If not, you'll get an error.
    std::vector<uint8_t> new_container;
    bool success = unpack_type(packet_out.data, new_container);

    if (success)
    {
        std::cout << std::endl;
        for (int i = 0; i<new_container.size(); i++)
        {
            std::cout << "Thing #" << i << "=" << new_container.at(i) << std::endl;
        }
    }
    else
    {
        std::cout << "Something went wrong unpacking your packet." << std::endl;
    }

    //std::vector<unsigned char> test_vec;
    //float asdf = -1256.123;
    //double asdf2 = -666.123456;

    // You can use something like this if you are on a bigEndian platform:
    //if (!LittleEndian())
    //    ByteSwap(&asdf, sizeof(float));
    /*
        push_back_type(test_vec, asdf);
        push_back_type(test_vec, asdf2);

        std::cout << std::endl << "Bytes in float" << std::endl;
        std::cout << "-----------------" << std::endl;
        int index = 0;
        unsigned char p[4];
        for (auto things : test_vec)
        {
            std::cout << index++ << "\t[" << (int)things << "]" << std::endl;
        }
        
        p[0] = test_vec.at(0);
        p[1] = test_vec.at(1);
        p[2] = test_vec.at(2);
        p[3] = test_vec.at(3);

        std::cout << (float)*(float*)p << std::endl;
        std::cout << "Float size = " << sizeof(float) << std::endl;
        std::cout << "Double size = " << sizeof(double) << std::endl;
        std::cout << LittleEndian() << std::endl;
    */

    //while (ros::ok())
    //{
    //    ros::spinOnce();
    //}
    return 0;
}