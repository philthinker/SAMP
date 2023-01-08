//vector2array16

#include <vector>
#include <array>

std::array<double,16> vector2array16( std::vector<double> vectorIn ){
    //vector2array Transform the 1-D vector of 16 elements to 1-D array
    std::array<double,16> arrayOut;
    for (unsigned int i = 0; i < 16; i++)
    {
        arrayOut[i] = vectorIn[i];
    }
    return arrayOut;
}