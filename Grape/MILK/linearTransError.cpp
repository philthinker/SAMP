//linearTransError
//  Calculate the error along a direction.

#include<cmath>
#include<Eigen/Dense>

double linearTransError(Eigen::Vector3d Pd, Eigen::Vector3d Pt, Eigen::Vector3d Vd)
{
    // Calculate the error along a direction.
    Eigen::Vector3d dP(Pd - Pt);
    double sign = dP.transpose()*Vd;
    sign = sign/dP.norm();
    double e = dP.norm();
    return sign * e;
}