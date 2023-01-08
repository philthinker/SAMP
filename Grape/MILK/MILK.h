//MILK

#include <Eigen/Dense>

double cosInterp(double x0, double xg, double t);
std::array<double,16> vectorP2arrayCarte(std::vector<double> vectorIn,std::array<double,16> carteIn);
Eigen::Vector3d quatSubtraction(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
Eigen::Matrix<double,6,6> vectorK2Matrix6(std::vector<double> k);
double linearTransError(Eigen::Vector3d Pd, Eigen::Vector3d Pt, Eigen::Vector3d Vd);
Eigen::Vector3d velRegulate(Eigen::Vector3d v, double THD);
Eigen::Matrix3d desiredFrameR(Eigen::Vector3d p0, Eigen::Vector3d pg, Eigen::Vector3d d);
Eigen::Vector3d goalUpdate0(Eigen::Vector3d Pgt, Eigen::Vector3d Pg, Eigen::Vector3d d, std::array<double,2> alpha);
Eigen::Vector3d goalUpdate1(Eigen::Vector3d Pgt, Eigen::Vector3d Pg, Eigen::Vector3d Pt, Eigen::Vector3d d, std::array<double,2> alpha);