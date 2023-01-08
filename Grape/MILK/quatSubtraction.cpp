//quatSubtraction
//  Compute the logrithmic map from q1 * bar_q2

#include <cmath>
#include <Eigen/Dense>

Eigen::Vector3d quatSubtraction(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
    //quatSubtraction
    /*
    if (q1.w() < 0.0)
    {
        q1.coeffs() = -q1.coeffs();
    }
    if (q2.w() < 0.0)
    {
        q2.coeffs() = -q2.coeffs();
    }
    */
    Eigen::Quaterniond q = q1 * q2.conjugate();
    if (q.w() < 0.0)
    {
        q.coeffs() = -q.coeffs();
    }
    Eigen::Vector3d eta;
    if (q.x() == 0.0 & q.y() == 0.0 & q.z() == 0.0)
    {
        eta.setZero();
    }else
    {
        double v = q.w();
        Eigen::Vector3d u = q.vec();
        eta = std::acos(v)*u/(u.norm());
    }
    return eta;
}