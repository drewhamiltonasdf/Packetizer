# Packetizer for ROS (Robot Operating System)

This repo has some modifications by Drew Hamilton to hidetaki's 
excellent Packetizer code.

This package allows you to easily create custom binary packets that
you can send between an Arduino/Teensy etc and a PC with almost no
overhead. The dream is to be able to send data in binary instead of
using strings because it is much much faster and requires your MCU
to spend less time processing serial data and more time doing interesting
things with it. It is also an obscene pain in the ass to come up with
a new string-parsing system everytime you want to write some new Arduino
code.

Here's an example of the savings for sending 4 floats. Lets take the following
array as an example{-123.66, 12345.98, -88.99002, 87654.99}:

            -Binary: 16 bytes
            
            -String: 38 bytes + '/r/n' = 40 bytes

If you've used strings before, you know the above example is really a best-case
scenario. So why not just send binary data blindly? Well, the difficulty with 
sending binary data is that you end up with carriage-returns, null-characters,
end-of-line chars etc all over the place. Where does your packet begin and
where does it end? COBS formatting solves that problem, and my utilities help
by making it easy to serialize a bunch of different data types into a binary
byte array. With the two, you should be able to send just about any custom
packet you want with speed and no bandwidth or buffering problems...

I recently added some functionality for packing floats/doubles etc into
the packets. Check out the example code ros_example.cpp.

Please check out ros_example.cpp and ros_example2.cpp
As long as you take your packets apart the same way you put them together
and you pay some attention to the way I've constructed packets in the 
examples you'll be good to go. These examples make it possible to test
the code out on a PC, so you don't need to send and recieve. The 
actual serial communication is up to you (at the moment).

Here's a snippet:
```
    Packetizer::Packet packet_in;
    packet_in.index = JOINT_POSITION_PACKET;

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

    std::cout << std::endl << "MCU Flags: "; printFlags(receive_flags);

    std::cout << std::endl << "Joint Positions after unpacking: " << std::endl;
    int i = 0;
    for (auto joint_pos : receive_JOINTS)
    {
        std::cout << "  [" << i++ << "]: " << std::setprecision(5) << joint_pos << std::endl;
    }
    std::cout << std::endl;
```

I'm using this with CRC between ROS and a Teensy 3.6 and it works great. 

Thanks, hideataki, great code. Please read all of their README stuff below...

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

CRC8 will be calculated including index byte.
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
