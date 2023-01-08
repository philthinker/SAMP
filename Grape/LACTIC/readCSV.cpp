//readCSV

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::vector<double>> readCSV(std::string fileName){
    //readCSV Read vector data from a .csv file of given fileName.csv
    fileName.append(".csv");
    std::vector<std::vector<double>> dataRead;
    std::ifstream fileIn(fileName,std::ios::in);
    if(fileIn.fail()){
        std::cerr << "File: " << fileName << " not found" << std::endl;
        throw std::exception();
        return dataRead;
    }
    std::string line;
    while (std::getline(fileIn,line) && fileIn.good())
    {
        std::istringstream dataIn(line);
        std::string dataTmp;
        std::vector<double> dataReadTmp;
        while (std::getline(dataIn,dataTmp,','))
        {
            dataReadTmp.push_back(std::stod(dataTmp));
        }
        dataRead.push_back(dataReadTmp);
    }
    std::cout << fileName << " read" << std::endl;
    return dataRead;
}