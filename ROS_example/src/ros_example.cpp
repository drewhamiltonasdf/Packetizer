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
    

    // Here are two examples. In the first example (commented out) we unpack our packet into doubles.
    // In the second example we use a custom struct with two different datatypes in it.
    // You can change the template type and get any kind of type you want as long as that's
    // what you made the packet out of in the first place. In other words, if you have a datatype that is 4 bytes long
    // your packet ought to be divisible by 4. If not, you'll get an error.
    // See note below about the dangers of using structs between platforms. I merely include the
    // example to highlight that these are template functions that work with any data type.

    // You can use something like this if you are on a bigEndian platform. Both of these funcs are defined in type_packer.h
    // if (!LittleEndian())
    //    ByteSwap(&asdf, sizeof(float));   

    //Here's and example using just doubles
    //Add in a non-char sized thing to our packet using this function defined in type_packer.h
    double asdf1 = -123.666; 
    double asdf2 = 42.978; 
    double asdf3 = -11128.000001; 
    int val1 = 123456;
    int val2 = 654321;
    
    push_back_type(packet_in.data, asdf1);
    push_back_type(packet_in.data, asdf2);
    push_back_type(packet_in.data, asdf3);

    push_back_type(packet_in.data, val1);
    push_back_type(packet_in.data, val2);
    
    std::vector<double> double_container(3);      //<-- This is what we'll unpack our data into.
    std::vector<int> int32_container(2);          //<-- Size your containers appropriately, or you will be reading non-sense

    //int32_container.resize(2);
    //double_container.resize(3);                //Initialize for 3 doubles

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
    
    // Demonstration of unpacking into std::vectors
    // the function is templated, so it will take any
    // kind of vector, but you need to resize() the vector
    // to tell the function how many to grab from the packet
    int pos0 = 0;
    int pos1 = double_container.size() * sizeof(double);
    
    unpack_type(packet_out.data, pos0, double_container);
    unpack_type(packet_out.data, pos1, int32_container);

    // Demonstration of how to unpack single values of basic data types
    // and c-style arrays of basic data types
    double single_val;
    double two_vals[2];

    unpack_type(packet_out.data, pos0, single_val);
    unpack_type(packet_out.data, pos0, two_vals, 2);

    std::cout << "doubles: " << std::endl;
    for (auto thing : double_container)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;

    std::cout << "int32_t's: " << std::endl;
    for (auto thing : int32_container)
    {
        std::cout << thing << std::endl;
    }
    std::cout << std::endl;

    std::cout << "single val = " << single_val << std::endl;
    std::cout << "array vals = " << two_vals[0] << ", " << two_vals[1] << std::endl;

    //while (ros::ok())
    //{
    //    ros::spinOnce();
    //}
    return 0;
}