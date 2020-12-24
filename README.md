# Packetizer for ROS (Robot Operating System)

This repo has some modifications by Drew Hamilton to hidetaki's 
excellent Packetizer code for embedded.

Additions mostly include addition of some #defines and CMakeLists
so the package can be built using ROS and used as a library in 
roscpp nodes on a PC.

Recently added some functionality for packing floats/doubles etc into
the packets. Check out the example code ros_example.cpp.

Please check out ros_example.cpp if you want to see what this can do.
I leave it completely up to you to make sure that you are communicating
between platforms that compile structs with the same memory layout.
If you want to be really safe, just send standard data types (doubles, floats etc)

Here's a snippet:
```
            struct Goober
            {
                char label[15];     // Don't use Strings etc. use fixed length char arrays
                uint8_t flags = 0;  // I have a few functions for setting individual bits in a byte, see below
                double d;
                float f;
            };

            Goober a; strcpy(a.label, "Label1"); a.d = -11128.0001; a.f = 12.99;
            Goober b; strcpy(b.label, "next1"); b.d = -21128.0001; b.f = 22.99;
            Goober c; strcpy(c.label, "helloworld!"); c.d = -31128.0001; c.f = 32.99;
            
            a.flags = 0b00000001;            
            
            setFlag(b.flags, 3, true);
            setFlag(b.flags, 4, true);

            push_back_type(packet_in.data, a);
            push_back_type(packet_in.data, b);
            push_back_type(packet_in.data, c);
            
                ...
            
            const auto& p_buff = Packetizer::encode(packet_in.index, packet_in.data.data(), packet_in.data.size(), USE_CRC);
            
            std::vector<Goober> new_container;
            bool success = unpack_type(packet_out.data, new_container);
            
            if (success)
            {
                std::cout << std::endl;
                for (int i = 0; i<new_container.size(); i++)
                {
                    std::cout << "Thing #" << i << "=" << new_container.at(i).d << ",\t\t" << new_container.at(i).f << ",\t\tLabel: " << new_container.at(i).label << std::endl;
                    printFlags(new_container.at(i).flags);
                }
            }
            
            
```

I'm using this with CRC between ROS and a Teensy 3.6 and it works great.

Thanks, hideataki, great code.

# Install / Use

In a catkin workspace

```
git clone https://github.com/drewhamiltonasdf/Packetizer.git
catkin_make
rosrun packetizer packetizer_example
```

This is a very barebones ROS example. It doesn't even use ROS_INFO, it uses std::cout.
There is also no handling of the actual serial communication. I'm doing that with
Boost. I intended for this to have no extra bloat.

binary data packetization encoder / decoder based on COBS / SLIP encoding


## Feature

- encode / decode binary arrays to COBS / SLIP packet protocol
- one-line packetizing and sending with Serial (or other Stream class)
- callback registration with lambda and automatic execution
- optionally following features are available
  - packet verification using crc8
  - send / subscribe data with index to indentify packet type
- this library is embeded and used inside of my other libraries
  - [MsgPacketizer](https://github.com/hideakitai/MsgPacketizer) : recommend to use this to serialize any value type


## Packet Protocol

### COBS (Consistent Overhead Byte Stuffing)

- default encoding of this library
- refer [here](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) for more detail
- this library follows encoding described above, note difference pointed out in [this discussion](https://medium.com/@circuit4us/coding-consistent-overhead-byte-stuffing-cobs-for-packet-data-e60d7a361cf)


### SLIP (Serial Line Internet Protocol)

- optional encoding of this library
- refer [here](https://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol) for more detail


### Index and CRC8 Option

You can optionally add index byte to idetify packet type, and crc8 byte to verify packet structure.
These options are disabled by default.
If you use them, original data array is modified like:

| index  | data    | crc8   |
| ------ | ------- | ------ |
| 1 byte | N bytes | 1 byte |

CRC8 will be calcurated including index byte.
After that, these byte arrays will be coverted to COBS / SLIP encoding.


## Usage

### COBS Encoding

``` C++
#include <Packetizer.h>

void setup()
{
    Serial.begin(115200);

    // register callback called if packet has come
    Packetizer::subscribe(Serial,
        [](const uint8_t* data, const size_t size)
        {
            // one-line send data array
            Packetizer::send(Serial, data, size);
        }
    );
}

void loop()
{
    Packetizer::parse(); // should be called to trigger callback
}

```

### Use SLIP Encoding

To use SLIP encoding by default, just define this macro before you `#include <Packetizer.h>`.

```C++
#define PACKETIZER_SET_DEFAULT_ENCODING_SLIP
```

### Enable Index and CRC8 Options

To enable indexing and verifying packet by crc8, define following macros each.

``` C++
// define these macros before include, to enable indexing and verifying
#define PACKETIZER_USE_INDEX_AS_DEFAULT
#define PACKETIZER_USE_CRC_AS_DEFAULT

#include <Packetizer.h>

uint8_t recv_index = 0x12;
uint8_t send_index = 0x34;

void setup()
{
    Serial.begin(115200);

    // you can add callback depending on index value
    Packetizer::subscribe(Serial, recv_index,
        [&](const uint8_t* data, const size_t size)
        {
            Packetizer::send(Serial, send_index, data, size); // send back packet
        }
    );

    // you can also add callback called every time packet comes
    Packetizer::subscribe(Serial,
        [&](const uint8_t index, const uint8_t* data, const size_t size)
        {
            // send back to same index
            Packetizer::send(Serial, index, data, size);
        }
    );
}

void loop()
{
    Packetizer::parse(); // automatically incoming packets are verified by crc
}
```


### Just Encoding

Just to encode / decode packets, you can use global mathod like:

```C++
Packetizer::Packet p_in {0xAB, {0x11, 0x22, 0x00, 0x33}}; // {index, {data}}
const auto& p_buff = Packetizer::encode(p_in.index, p_in.data.data(), p_in.data.size());
const auto& p_out = Packetizer::decode(p_buff.data.data(), p_buff.data.size());
```

`Packetizer::Packet` is alias for `struct { uint8_t index; std::vector<uint8_t> data };`.


## Other Options

### Packet Data Storage Class Inside

STL is used to handle packet data by default, but for following boards/architectures, [ArxContainer](https://github.com/hideakitai/ArxContainer) is used to store the packet data because STL can not be used for such boards.
The storage size of such boards for packets, queue of packets, max packet binary size, and callbacks are limited.

- AVR
- megaAVR
- SAMD


### Memory Management (for NO-STL Boards)

As mentioned above, for such boards like Arduino Uno, the storage sizes are limited.
And of course you can manage them by defining following macros.
But these default values are optimized for such boards, please be careful not to excess your boards storage/memory.

``` C++
// max number of decoded packet queues
#define PACKETIZER_MAX_PACKET_QUEUE_SIZE     1
// max data bytes in packet
#define PACKETIZER_MAX_PACKET_BINARY_SIZE  128
// max number of callback for one stream
#define PACKETIZER_MAX_CALLBACK_QUEUE_SIZE   4
// max number of streams
#define PACKETIZER_MAX_STREAM_MAP_SIZE       2
```

For other STL enabled boards, only max packet queue size can be changed.
Default value is 0 and not limited.

``` C++
#define PACKETIZER_MAX_PACKET_QUEUE_SIZE 3 // default: 0
```


## Dependencies

### Teensy 3.x on Arduino IDE

- Please follow this instruction ([TeensyDirtySTLErrorSolution](https://github.com/hideakitai/TeensyDirtySTLErrorSolution)) to enable STL


### Other Platforms

None


## Embedded Libraries

- [ArxTypeTraits v0.2.1](https://github.com/hideakitai/ArxTypeTraits)
- [ArxContainer v0.3.10](https://github.com/hideakitai/ArxContainer)
- [ArxSmartPtr v0.2.1](https://github.com/hideakitai/ArxSmartPtr)
- [CRCx v0.2.1](https://github.com/hideakitai/CRCx)
- [TeensyDirtySTLErrorSolution v0.1.0](https://github.com/hideakitai/TeensyDirtySTLErrorSolution)


## Used Inside of

- [MsgPacketizer](https://github.com/hideakitai/MsgPacketizer)
- [ES920](https://github.com/hideakitai/ES920)


## License

MIT
