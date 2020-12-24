#ifndef TYPE_PACKER_H
#define TYPE_PACKER_H

#include <stdint.h>
#include <iostream>         //Only used by printFlags, comment out if you like
#include <vector>

//Set flag at position x to true/false
void setFlag(uint8_t& flag_char, uint8_t pos, bool bit_val) { flag_char ^= (-bit_val ^ flag_char) & (1UL << pos); }

//Flip flag at position
void toggleFlag(uint8_t& flag_char, uint8_t pos) { flag_char ^= 1UL << pos; }

//Get flag value at specified bit position
bool getFlagAt(uint8_t& flag_char, uint8_t pos) {  return (flag_char >> pos) & 1U; }

//For debugging, cout a flags bits
void printFlags(uint8_t& flag_char) { for (int i = 0; i < 8; i++) {  std::cout << "Flag #" << i << " = " << getFlagAt(flag_char, i); } }

//Append one item of type T to a std::vector<char>
//for example, break a float (4 bytes) into 4 chars and push_back each
//similarly a double (8 bytes) into 8 chars.
//char into the vector
//don't be an idiot and try to append some complicated container that
//uses pointers etc. behind the scenes. If you try to append an entire
//std::vector for example, you'll end up with a bunch of chars for the
//memory addresses of things. etc. etc.
template<typename T>
void push_back_type(std::vector<unsigned char>& vec, T& thing)
{
        unsigned char *p = (unsigned char *)&thing;
        
        for (int i = 0; i < sizeof(T); i++)
        {
            vec.push_back(p[i]);
        }
}

template<typename T>
bool unpack_type(const std::vector<unsigned char>& raw_buff, std::vector<T>& new_container)
{
    if (raw_buff.size() % sizeof(T) == 0) 
    {
        for (int data_position = 0; data_position < (raw_buff.size() / sizeof(T)) ; data_position++)
        {
            uint8_t temp[sizeof(T)];
            for (unsigned int i = 0; i < sizeof(T); i++ )
            {
                temp[i] = (uint8_t)(raw_buff[data_position*sizeof(T)+i]);
            }
            new_container.push_back(*((T *) &temp[0]));
        }
        return true;
    }
    else
    {
        throw std::runtime_error ("\nYour input container is not divisible by the number of bytes in the type specified in by your target container.\n");
        return false;
    }
    
}

void ByteSwap(void * data, int size)
{
    char * ptr = (char *) data;
    for (int i = 0;  i < size/2;  ++i)
        std::swap(ptr[i], ptr[size-1-i]);
}

bool LittleEndian()
{
    int test = 1;
    return *((char *)&test) == 1;
}

///  // splits a 32bit into 4 8bits
 /*
rightshifty32 (int16_t argn)
{
unsigned char bytes[4];
bytes[0] = (argn >> 24) & 0xFF;     //i = 0     sizeof(T) = 4       sizeof(T)-i-1 = 3     (sizeof(T)-i-1) * 8 = 24
bytes[1] = (argn >> 16) & 0xFF;     //i = 1     sizeof(T) = 4       sizeof(T)-i-1 = 2     (sizeof(T)-i-1) * 8 = 16
bytes[2] = (argn >> 8) & 0xFF;      //i = 2     sizeof(T) = 4       sizeof(T)-i-1 = 1     (sizeof(T)-i-1) * 8 = 8       
bytes[3] = argn & 0xFF;             //i = 3     sizeof(T) = 4       sizeof(T)-i-1 = 0     (sizeof(T)-i-1) * 8 = 0
printf("0x%x 0x%x 0x%x 0x%x\n", (unsigned char)bytes[0],
                        (unsigned char)bytes[1],
                        (unsigned char)bytes[2],
                        (unsigned char)bytes[3]);
}
*/

#endif