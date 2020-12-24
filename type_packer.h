#ifndef TYPE_PACKER_H
#define TYPE_PACKER_H

#include <stdint.h>
#include <iostream>         //Only used by printFlags, comment out if you like
#include <vector>

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

//Set flag at position x to true/false
void setFlag(uint8_t& flag_char, uint8_t pos, bool bit_val) { flag_char ^= (-bit_val ^ flag_char) & (1UL << pos); }

//Flip flag at position
void toggleFlag(uint8_t& flag_char, uint8_t pos) { flag_char ^= 1UL << pos; }

//Get flag value at specified bit position
bool getFlagAt(uint8_t& flag_char, uint8_t pos) {  return (flag_char >> pos) & 1U; }

//For debugging, cout a flags bits
void printFlags(uint8_t& flag_char) 
{ 
    //std::cout << std::endl;    
    for (int i = 0; i < 8; i++) 
    {
        std::cout << "\t[" << i << "]:" << (bool)getFlagAt(flag_char, i);  
    }
    std::cout << std::endl;
}

#endif