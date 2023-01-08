//Grape6
// The SAMP v6.0.0
//
// Haopeng Hu
// 2023.01.08
// All rights reserved

#include<Eigen/Dense>

#include<array>
#include<string>
#include<vector>
#include<iostream>
#include<fstream>
#include<sstream>
#include<cmath>

#include "Sweetness5S0.h"
#include "Redgrape10.h"

class Grape6
{
// Properties
public:
    Eigen::Vector3d p_l;          // Pre-alignment position
    Eigen::Quaterniond q_s0;      // Pre-assembly orientation
    Eigen::Vector3d p_s0;         // Pre-assembly position
    Eigen::Vector3d d_l;          // Alignment direction
    std::array<unsigned int,6> mod;          // Movement types of the SAMP
    std::array<Eigen::Vector3d,6> p_s;       // Positions of each AMP
    std::array<Eigen::Quaterniond,6> q_s;    // Roentations of each AMP
    std::array<Eigen::Vector3d,6> d_s;       // Translation directions of each AMP

    Redgrape10 cnt;               // Controller
    Eigen::Matrix3d Rsd;          // Desired frame

    unsigned int terminalMod;     // The terminal mode
                                  // 0/1/2/3 for normal/forceout/momentout/marginout
                                  // 4 for safe notification
    Sweetness5S0 pfm;
protected:
    unsigned int t;                 // Step counter
    unsigned int p;                 // Phase counter
    unsigned int ns;                // Total num. of AMPs
    unsigned int ts;                // AMP counter
private:
    double f_thd;                   // The touching force threshold
    double m_thd;                   // The touching moment threshold
    double f_max;                   // The upper bound of touch force
    std::array<double,2> kd;
    std::array<double,2> kc;
    std::array<double,2> dd;
    std::array<double,2> dc;
// Methods
public:
    Grape6();
    Grape6(std::string file_name, unsigned int FPS);
    ~Grape6();
    void print();
    void summary();
    unsigned int getCurrPhase();
    unsigned int getCurrState(bool typeFlag = false);

    void A1Init(Eigen::Affine3d T0, std::array<double,2> kd, std::array<double,2> dd);
    void A1GoalUpdate(unsigned int PERIOD, Eigen::Vector3d pt, std::array<double,2> coef, double scal);
    bool A1Terminal(Eigen::Vector3d pt, Eigen::Quaterniond qt, std::array<double,2> maxErr);
    bool A1BeyondMargin(Eigen::Vector3d pt, Eigen::Quaterniond qt, std::array<double,2> margin);

    void A2Init(Eigen::Vector3d p_t, Eigen::Quaterniond q_t, double kd_p, std::array<double,2> kc, double dd_p, std::array<double,2> dc);
    void A2GoalUpdate(Eigen::Matrix<double,6,1> f_ext, double step, std::array<double,2> coefs);
    bool A2Terminal(Eigen::Vector3d pt, double maxErr);
    bool A2BeyondMargin(Eigen::Vector3d pt, Eigen::Quaterniond qt, std::array<double,2> margin);

    void A3Init(Eigen::Vector3d pt, Eigen::Quaterniond qt,
        std::array<double,2> kd, std::array<double,2> kc,
        std::array<double,2> dd, std::array<double,2> dc);
    void A3GoalUpdate(Eigen::Matrix<double,6,1> f_ext, std::array<double,2> step, std::array<double,2> coefs);
    bool A3Terminal(Eigen::Vector3d pt, Eigen::Quaterniond qt, Eigen::Matrix<double,6,1> f_ext, std::array<double,2> maxErr, std::array<double,2> maxFM);
protected:
    std::vector<std::vector<double>> readCSV(std::string fileName);
    Eigen::Matrix3d setDiagMatrix(double k_x, double k_yz);

    void desiredFrameR(Eigen::Vector3d d);
    void desiredImpedance(unsigned int mod, std::array<double,2> kd, std::array<double,2> kc, std::array<double,2> dd, std::array<double,2> dc);
private:
    void defaultBehavior();
};

