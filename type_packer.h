#ifndef TYPE_PACKER_H
#define TYPE_PACKER_H

#include <stdint.h>
#include <iostream>         //Only used by printFlags, comment out if you like
#include <vector>


void ByteSwap(void * data, int size);

//Basic template for adding simple data types
template<typename T>
void push_back_type(std::vector<unsigned char>& vec, T& thing)
{
        unsigned char *p = (unsigned char *)&thing;
        
        for (int i = 0; i < sizeof(T); i++)
        {
            vec.push_back(p[i]);
        }
}


//Override for arrays.
template<typename T>
void push_back_type(std::vector<unsigned char>& vec, T* thing, int size)
{
    for (int pos = 0; pos < size; pos++)
    {
        unsigned char *p = (unsigned char *)&thing[pos];

        for (int i = 0; i < sizeof(T); i++)
        {
            vec.push_back(p[i]);
        }
    }
}


//Override for std::vectors.
template<typename T>
void push_back_type(std::vector<unsigned char>& vec, std::vector<T>& thing)
{
    for (auto discrete_thingamajig : thing)
    {
        push_back_type(vec, discrete_thingamajig);
    }
}


//Override for passing in const values. Careful with type deduction!
template<typename T>
void push_back_type_const(std::vector<unsigned char>& vec, const T thing)
{
        unsigned char *p = (unsigned char *)&thing;
        
        for (int i = 0; i < sizeof(T); i++)
        {
            vec.push_back(p[i]);
        }
}

//Variadic template recurses above functions so we can pass an endlessly long list of return containers
//template <typename T, typename... Rest>
//int unpack_type(const std::vector<unsigned char>& raw_buff, int pos, T &t, Rest&... rest) {
//  return unpack_type(raw_buff, unpack_type(raw_buff, pos, t), rest...);
//}

//Variadic template Push back a list of junk
template <typename T, typename... Rest>
void push_back_type(std::vector<unsigned char>& raw_buff, T &t, Rest&... rest) 
{
    push_back_type(raw_buff, t);
    push_back_type(raw_buff, rest...);
}


//This is for array returns (data types like float[], double[], int[] etc.)
template<typename T>
int unpack_type(const std::vector<unsigned char>& raw_buff, int pos, T *thing, int arr_size)
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
    return pos+(arr_size * sizeof(T));
}

//This is for single-value returns (data types like float, double, int etc.), passed by ref
template<typename T>
int unpack_type(const std::vector<unsigned char>& raw_buff, int pos, T &thing)
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
    return pos+(sizeof(T));
}

//This is for std::vector returns (std::vector<float>, std::vector<int> etc)
template<typename T>
int unpack_type(const std::vector<unsigned char>& raw_buff, int pos, std::vector<T>& new_container)
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
        
        return pos+(new_container.size() * sizeof(T));;
    
}

//Variadic template recurses above functions so we can pass an endlessly long list of return containers
template <typename T, typename... Rest>
int unpack_type(const std::vector<unsigned char>& raw_buff, int pos, T &t, Rest&... rest) {
  return unpack_type(raw_buff, unpack_type(raw_buff, pos, t), rest...);
}

//Version that takes no initial pos, and assumes zero
template <typename T, typename... Rest>
int unpack_type(const std::vector<unsigned char>& raw_buff, T &t, Rest&... rest) 
{
  int pos = 0;
  return unpack_type(raw_buff, unpack_type(raw_buff, pos, t), rest...);
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
        std::cout << "[" << i << "]:" << (bool)getFlagAt(flag_char, i) << "   ";  
    }
    std::cout << std::endl;
}

#endif