//getDataFromInput

#include<string>

double getDataFromInput(char* argv, double min_bound, double max_bound){
    // Get data from the input argument
    std::string dataIn(argv);
    double dataOut = std::stod(dataIn);
    if (dataOut < min_bound)
    {
        dataOut = min_bound;
    }else if (dataOut > max_bound)
    {
        dataOut = max_bound;
    }
    return dataOut;
}