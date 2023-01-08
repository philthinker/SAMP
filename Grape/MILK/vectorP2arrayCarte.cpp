//vectorP2arrayCarte
// Convert vector<double> to array<double,16> 

#include <vector>
#include <array>

std::array<double,16> vectorP2arrayCarte(std::vector<double> vectorIn,std::array<double,16> carteIn){
    std::array<double, 16> carteOut = carteIn;
    for (unsigned int i = 12; i < 15; i++)
    {
        carteOut[i] = vectorIn[i-12];
    }
    return carteOut;
}