//LACTIC.h

#include <vector>
#include <string>

std::vector<std::vector<double>> readCSV(std::string fileName);

double getDataFromInput(char* argv, double min_bound, double max_bound);

std::array<double,16> vector2array16( std::vector<double> vectorIn );

std::array<double,7> vector2array7( std::vector<double> vectorIn );