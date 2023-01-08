//velRegulate
//  Regulate the translation velocity to avoid too small value.

#include<Eigen/Core>

Eigen::Vector3d velRegulate(Eigen::Vector3d v, double THD)
{
    if (v.norm() < THD)
    {
        v.setZero();
    }else
    {
        v.normalize();
    }
    return v;
}