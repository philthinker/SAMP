//vector2array7

#include <vector>
#include <array>

std::array<double,7> vector2array7( std::vector<double> vectorIn )
{
    //vector2array Transform the 1-D vector of 7 elements to 1-D array
    std::array<double,7> arrayOut;
    for (unsigned int i = 0; i < 7; i++)
    {
        arrayOut[i] = vectorIn[i];
    }
    return arrayOut;
}