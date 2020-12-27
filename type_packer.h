#ifndef TYPE_PACKER_H
#define TYPE_PACKER_H

#include <stdint.h>
#include <iostream>         //Only used by printFlags, comment out if you like
#include <vector>


void ByteSwap(void * data, int size);

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

//This is for array returns (data types like float[], double[], int[] etc.)
template<typename T>
bool unpack_type(const std::vector<unsigned char>& raw_buff, int &pos, T *thing, int arr_size)
{
    bool overflow = false;
    for (int data_position = 0; data_position < arr_size; data_position++)
    {
        uint8_t temp[sizeof(T)];
        for (unsigned int i = 0 + pos; i < sizeof(T) + pos; i++ )
        {
            if (i < raw_buff.size()) { temp[i-pos] = (uint8_t)(raw_buff[data_position*sizeof(T)+i]); }
            else {overflow = true;}
        }
        if (!overflow) {
            thing[data_position] = (*((T *) &temp[0]));
        }
    }
    return true;
}

//This is for single-value returns (data types like float, double, int etc.)
template<typename T>
bool unpack_type(const std::vector<unsigned char>& raw_buff, int &pos, T &thing)
{
    bool overflow = false;
    uint8_t temp[sizeof(T)];
    for (unsigned int i = 0 + pos; i < sizeof(T) + pos; i++ )
    {
        if (i < raw_buff.size()) { temp[i-pos] = (uint8_t)(raw_buff[i]); }
        else {overflow = true;}
    }
    if (!overflow) {
        thing = (*((T *) &temp[0]));
    }
    return true;
}

//This is for std::vector returns (std::vector<float>, std::vector<int> etc)
template<typename T>
bool unpack_type(const std::vector<unsigned char>& raw_buff, int &pos, std::vector<T>& new_container)
{
        bool overflow = false;
        for (int data_position = 0; data_position < new_container.size(); data_position++)
        {
            uint8_t temp[sizeof(T)];
            for (unsigned int i = 0 + pos; i < sizeof(T) + pos; i++ )
            {
                if (i < raw_buff.size()) { temp[i-pos] = (uint8_t)(raw_buff[data_position*sizeof(T)+i]); }
                else {overflow = true;}
            }
            if (!overflow) {
                new_container.at(data_position) = (*((T *) &temp[0]));
            }
        }
        
        return overflow;
    
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