//Sweetness5S0
// The Performance Evaluator for A3 Framework. v5S.0
//
//  Haopeng Hu
//  2022.05.11
//  All rights reserved.
//

#ifndef _Sweetness5S0_
#define _Sweetness5S0_

#include<Eigen/Dense>

#include<array>
#include<string>
#include<vector>
#include<iostream>
#include<fstream>
#include<sstream>
#include<cmath>

class Sweetness5S0
{
    public:
    Sweetness5S0();
    ~Sweetness5S0();
    Sweetness5S0(unsigned int fps);

    // Data
    std::array<unsigned int, 3> Dura;   // Durations of each phase.
    unsigned int tTotal;                // Total time taken.
    Eigen::Matrix<double,6,2> maxFT;    // Max. F/T in A2 and A3.
    bool successFlag;                   // true for task acomplished.
    // Methods
    void record(unsigned int t, unsigned int p);  // Record the performance indicators.
    void record(unsigned int t, unsigned int p, std::array<double,6> f_ext);
    void record(unsigned int t, unsigned int p, unsigned int s,
        std::ofstream& log_file,
        std::array<double,16> pose, std::array<double,6> f_ext);
    void record(unsigned int t, unsigned int p, unsigned int s,
        std::ofstream& log_file,
        Eigen::Vector3d Pt, Eigen::Vector3d Pgt,
        Eigen::Quaterniond Qt, Eigen::Quaterniond Qgt,
        std::array<double,6> f_ext);
    void record(unsigned int p, unsigned int s,
        std::ofstream& log_file,
        Eigen::Vector3d Pt, Eigen::Vector3d Pgt,
        Eigen::Quaterniond Qt, Eigen::Quaterniond Qgt,
        std::array<double,6> f_ext);
    void record(unsigned int t, unsigned int p, unsigned int s,
        std::ofstream& log_file,
        Eigen::Vector3d Pt, Eigen::Quaterniond Qt,
        std::array<double,6> f_ext, std::array<double,7> tau_c);
    void summary(bool flag = true); // Summarize the indicators.
    void print();   // Print thee indicators.
    void print(Eigen::Vector3d f_t, Eigen::Vector3d m_t);
    void print(double e);
    bool tick(bool seconds = false);

    protected:
    unsigned int Period;           // 1000/FPS
    
    private:
    unsigned int stepCounter;           // Last step counter
    unsigned int logCounter;            // The log counter
    bool summaryFlag;                   // True for summarized
};

#endif