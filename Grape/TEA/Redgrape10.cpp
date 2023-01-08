//Redgrape10

#include "Redgrape10.h"

Redgrape10::Redgrape10()
{
    K.setZero();
    D.setZero();
    Pgt.setZero();
    Qgt.setIdentity();
}

Redgrape10::~Redgrape10()
{
}

// Read a quaternion from a vector of [w x y z]
Eigen::Quaterniond Redgrape10::quatFromVector(std::vector<double> q)
{
    Eigen::Quaterniond Q;
    Q.w() = q[0];
    Q.x() = q[1];
    Q.y() = q[2];
    Q.z() = q[3];
    Q.normalize();
    return Q;
}

// Product two quaternions (Same to Eigen maybe)
Eigen::Quaterniond Redgrape10::quatProduct(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
    Eigen::Matrix<double,4,4> q1hat;
    q1hat(0,0) = q1.w(); q1hat(0,1) = -q1.x(); q1hat(0,2) = -q1.y(); q1hat(0,3) = -q1.z();
    q1hat(1,0) = q1.x(); q1hat(1,1) = q1.w(); q1hat(1,2) = -q1.z(); q1hat(1,3) = q1.y();
    q1hat(2,0) = q1.y(); q1hat(2,1) = q1.z(); q1hat(2,2) = q1.w(); q1hat(2,3) = -q1.x();
    q1hat(3,0) = q1.z(); q1hat(3,1) = -q1.y(); q1hat(3,2) = q1.x(); q1hat(3,3) = q1.w();
    Eigen::Matrix<double,4,1> q2vec;
    q2vec(0,0) = q2.w(); q2vec(1,0) = q2.x(); q2vec(2,0) = q2.y(); q2vec(3,0) = q2.z();
    Eigen::Matrix<double,4,1> qProduct = q1hat * q2vec;
    Eigen::Quaterniond q;
    q.w() = qProduct(0,0);
    q.x() = qProduct(1,0);
    q.y() = qProduct(2,0);
    q.z() = qProduct(3,0);
    return q;
}

// Quaternion log map
Eigen::Vector3d Redgrape10::quatLogMap(Eigen::Quaterniond q, Eigen::Quaterniond qa)
{
    Eigen::Vector3d eta;
    Eigen::Quaterniond dq = Redgrape10::quatProduct(q,qa.conjugate());
    double w = std::acos(dq.w());
    eta = dq.vec();
    if (eta.norm() < 0.000001)
    {
        eta.setZero();
    }else
    {
        eta = w*(eta/eta.norm());
    }
    return eta;
}

// Quaternion exp map
Eigen::Quaterniond Redgrape10::quatExpMap(Eigen::Vector3d eta)
{
    Eigen::Quaterniond q(Eigen::Quaterniond::Identity());
    if (eta.norm() > 0.000001)
    {
        double w = eta.norm();
        q.w() = std::cos(w);
        q.vec() = std::sin(w)*(eta/w);
    }
    if (q.w() < 0)
    {
        q.coeffs() = -q.coeffs();
    }
    return q;
}

// Anle error between unit quaternions.
double Redgrape10::quatError(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
    Eigen::Vector3d eta = Redgrape10::quatLogMap(q1,q2);
    double theta = 2*eta.norm();
    const double pi = 3.15159;
    if (theta > pi)
    {
        // Obotus angle 
        theta = 2*pi - theta;
    }
    return theta;
}

// Substract the quaternion
Eigen::Vector3d Redgrape10::quatSubstract(Eigen::Quaterniond q1, Eigen::Quaterniond q2, bool flagLog)
{
    // flagLog: true for using the Redgrape10::quatLogMap method
    Eigen::Vector3d eta;
    if (flagLog)
    {
        // Same to the Redgrape10::quatLogMap method
        eta = quatLogMap(q1,q2);
    }else
    {
        Eigen::Quaterniond q = q1 * q2.conjugate();
        if (q.w() < 0.0)
        {
            q.coeffs() = -q.coeffs();
        }
        if (q.vec().norm() < 0.00001)
        {
            eta.setZero();
        }else
        {
            double v = q.w();
            double theta = 2*std::acos(v);
            if (theta >= 3.1416)
            {
                theta = theta - 2*3.1416;
            }
            Eigen::Vector3d u = q.vec();
            eta = theta*u/(u.norm());
        }
    }
    return eta;
}

// Update the goal orientation (unit quaternion) given angular velocity
Eigen::Quaterniond Redgrape10::quatUpdate(Eigen::Quaterniond q0, Eigen::Vector3d w, double dt)
{
    // q(t+dt) = exp((dt/2)w)q(t)
    Eigen::Quaterniond dq = quatExpMap((dt/2)*w);
    return dq*q0;
}

// Adjoint matrix
Eigen::Matrix<double,6,6> Redgrape10::adjoint(Eigen::Affine3d T)
{
    Eigen::Matrix<double,6,6> AdjT(Eigen::MatrixXd::Identity(6,6));
    Eigen::Matrix3d R(T.linear());
    Eigen::Vector3d p(T.translation());
    AdjT.topLeftCorner(3,3) << R;
    AdjT.bottomRightCorner(3,3) << R;
    AdjT.topRightCorner(3,3) << skewSymmetric(p) * R;
    return AdjT;
}

// Adjoint matrix
Eigen::Matrix<double,6,6> Redgrape10::adjoint(Eigen::Matrix3d R, Eigen::Vector3d p)
{
    // Never forget the Franka twist and wrench are defined as 
    // [v; w] and [f; m] respectively.
    // Hence, the adjoint form is [R, p^R; 0, R].
    Eigen::Matrix<double,6,6> AdjT(Eigen::MatrixXd::Identity(6,6));
    AdjT.topLeftCorner(3,3) << R;
    AdjT.bottomRightCorner(3,3) << R;
    AdjT.topRightCorner(3,3) << skewSymmetric(p) * R;
    return AdjT;
}

// Skew symmetric matrix of the 3d vector.
Eigen::Matrix3d Redgrape10::skewSymmetric(Eigen::Vector3d v)
{
    Eigen::Matrix3d vhat; vhat.setZero();
    vhat(0,1) = -v(2); vhat(0,2) = v(1); vhat(1,2) = -v(0);
    return vhat - vhat.transpose();
}
