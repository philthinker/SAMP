//Redgrape10
//  The impedance controller parameters. v1.0
//
//  Haopeng Hu
//  2022.05.13
//  All rights reserved

#ifndef _Redgrape10_
#define _Redgrape10_

#include<Eigen/Dense>

#include<array>
#include<vector>
#include<iostream>
#include<cmath>

class Redgrape10
{
    public:
    Redgrape10();
    ~Redgrape10();
    // Controller param.
    Eigen::Matrix<double,6,6> K;    // Stiffness
    Eigen::Matrix<double,6,6> D;    // Damping
    // Loop variable
    Eigen::Vector3d Pgt;            // Current goal position
    Eigen::Quaterniond Qgt;         // Current goal orientation
    
    // Static methods
    static Eigen::Quaterniond quatFromVector(std::vector<double> q);
    static Eigen::Vector3d quatLogMap(Eigen::Quaterniond q, Eigen::Quaterniond qa = Eigen::Quaterniond::Identity());
    static Eigen::Quaterniond quatExpMap(Eigen::Vector3d eta);
    static Eigen::Quaterniond quatProduct(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
    static double quatError(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
    static Eigen::Vector3d quatSubstract(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool flagLog = true);
    static Eigen::Quaterniond quatUpdate(Eigen::Quaterniond q0, Eigen::Vector3d w, double dt = 1);
    static Eigen::Matrix<double,6,6> adjoint(Eigen::Affine3d T);
    static Eigen::Matrix<double,6,6> adjoint(Eigen::Matrix3d R, Eigen::Vector3d p);
    static Eigen::Matrix3d skewSymmetric(Eigen::Vector3d v);
};

#endif