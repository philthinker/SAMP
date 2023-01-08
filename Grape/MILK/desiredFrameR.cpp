//desiredFrameR
//  Calculate the rotation transformation of the desired desired frame
//
//  Haopeng Hu
//  2022.03.16
//
//  p0: the initial position
//  pg: the goal position
//  d: the desired direction (unit)

#include<Eigen/Dense>

Eigen::Matrix3d desiredFrameR(Eigen::Vector3d p0, Eigen::Vector3d pg, Eigen::Vector3d d)
{
    Eigen::Vector3d e(pg - p0);
    Eigen::Vector3d yd(e - (e.dot(d))*d); yd.normalize();
    Eigen::Vector3d zd(d.cross(yd));
    Eigen::Matrix3d Rsd; Rsd.setZero();
    Rsd.col(0) << d;
    Rsd.col(1) << yd;
    Rsd.col(2) << zd;
    return Rsd;
}
