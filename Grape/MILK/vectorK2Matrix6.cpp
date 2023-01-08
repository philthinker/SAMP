//vectorK2Matrix6
//  Transit the value of vector<double> to Matrix<double,6,6>

#include <vector>
#include <Eigen/Core>

Eigen::Matrix<double,6,6> vectorK2Matrix6(std::vector<double> k)
{
    //vector2Matrix6 
    Eigen::Matrix<double,6,6> K;
    K.setZero();
    for (unsigned int i = 0; i < 6; i++)
    {
        K(i,i) = k[i];
    }
    return K;
}
