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

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "rosnode_created_in_vs_code");
    //ros::NodeHandle n;


    Packetizer::Packet packet_in;
    packet_in.index = 123;

    double asdf1 = -123.666; 
    double asdf2 = 42.978; 
    double asdf3 = -11128.000001; 
    int val1 = 123456;
    int val2 = 654321;
    float float_array[3] = {-12.123, 2.991, 666};
    std::vector<char> char_container{ 'Z', 'O', 'X' };
    char blooper = 'Z';
    
    /* // These are examples of pushing one value at a time onto the packet
    push_back_type(packet_in.data, asdf1);              //Push back individual doubles!
    push_back_type(packet_in.data, asdf2);
    push_back_type(packet_in.data, asdf3);
    push_back_type(packet_in.data, val1);               //Push back individual ints!
    push_back_type(packet_in.data, val2);               
    push_back_type(packet_in.data, float_array, 3);     //Push back arrays!
    push_back_type(packet_in.data, char_container);     //Push back std::vectors
    push_back_type_const(packet_in.data, 'Z');          //Push back const things in line just because
    */

    // But let's do better... here we pack everything on at the same time, so we can see very explicitly what
    // the packet will be composed of. The only issue with this approach is that it doesn't work super well with arrays
    // because there's no place to show what size the arrays are.
    push_back_type(packet_in.data, asdf1, asdf2, asdf3, val1, val2, float_array[0], float_array[1], float_array[2], char_container, blooper);

    std::vector<double> receive_doubles(3);      //<-- This is what we'll unpack our data into.
    int receive_int32s[2];                       //<-- Size your containers appropriately, or you will be reading non-sense
    char receive_single_char;                    //<-- receive vars can be any basic data types as well
    std::vector<float> receive_floats(3);
    std::vector<char> receive_chars(3);     

    const auto& p_buff = Packetizer::encode(packet_in.index, packet_in.data.data(), packet_in.data.size(), USE_CRC);
    const auto& packet_out = Packetizer::decode(p_buff.data.data(), p_buff.data.size(), USE_INDEX, USE_CRC);

    //Notice I don't have a great way to get the size of arrays, that's why I'm using std containers
    int tracker = unpack_type(packet_out.data, receive_doubles, receive_int32s[0], receive_int32s[1], receive_floats, receive_chars, receive_single_char);

    std::cout << "doubles after unpacking: " << std::endl;
    for (auto thing : receive_doubles)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;
    std::cout << "int32s after unpacking: " << std::endl;
    for (auto thing : receive_int32s)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;
    std::cout << "floats after unpacking: " << std::endl;
    for (auto thing : receive_floats)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;
    std::cout << "chars after unpacking: " << std::endl;
    for (auto thing : receive_chars)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;    
    std::cout << "receive_single_char = " << receive_single_char << std::endl;

    //inline const Packet& decode(const uint8_t* data, const size_t size, const bool b_index, const bool b_crc)
    /*
    std::cout << "input   = " << std::endl << std::endl; for (const auto& p : packet_in.data) { std::cout << "[" << p << "]: " << (int)p << std::endl;  }  std::cout << std::endl;
    std::cout << "encoded = " << std::endl << std::endl; for (const auto& p : p_buff.data) {std::cout << "[" << p << "]: " << (int)p << std::endl;  }  std::cout << std::endl;
    std::cout << "decoded = " << std::endl << std::endl; for (const auto& p : packet_out.data) {std::cout << "[" << p << "]: " << (int)p << std::endl;  } std::cout << std::endl;
    std::cout << "index_out = [" << (int)packet_out.index << "]" << std::endl;
    if (packet_in.data != packet_out.data) {std::cout << "test 1 failed!"<< std::endl;} else {std::cout << std::endl << "test 1 success!"<< std::endl;}
    std::cout << "CRC is expected to be = " << (int)crcx::crc8(packet_in.data.data(), packet_in.data.size()) << std::endl;
    std::cout << "CRC from encoded packet is = " << (int)p_buff.data.at(static_cast<int>(p_buff.data.size()) - 2) << std::endl;
    */

    //UNPACKING THE SLOW WAY:
    /*
    // Demonstration of unpacking into std::vectors
    // the function is templated, so it will take any
    // kind of vector, but you need to resize() the vector
    // to tell the function how many to grab from the packet
    int pos0 = 0;
    int pos1 = receive_doubles.size() * sizeof(double);td::vector<int>
    unpack_type(packet_out.data, pos1, receive_int32s);

    // Demonstration of how to unpack single values of basic data types
    // and c-style arrays of basic data types
    double single_val;
    double two_vals[2];
    char single_char;

    unpack_type(packet_out.data, pos0, single_val);
    unpack_type(packet_out.data, pos0, two_vals, 2);
    unpack_type(packet_out.data, packet_out.data.size() - 1, single_char);

    std::cout << "doubles: " << std::endl;
    for (auto thing : receive_doubles)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;

    std::cout << "int32_t's: " << std::endl;
    for (auto thing : receive_int32s)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;

    std::cout << "single val = " << single_val << std::endl;
    std::cout << "array vals = " << two_vals[0] << ", " << two_vals[1] << std::endl;
    std::cout << "single char = " << single_char << std::endl;
    */

    //while (ros::ok())
    //{
    //    ros::spinOnce();
    //}
    return 0;
}