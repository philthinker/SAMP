//Sweetness5S0
// The Performance Evaluator for A3 Framework. v5S.0
//
//  Haopeng Hu
//  2022.05.11
//  All rights reserved.
//

#include "Sweetness5S0.h"

Sweetness5S0::Sweetness5S0()
{
    Dura = {{0, 0, 0}};
    tTotal = 0;
    Period = 1;
    stepCounter = 0;
    logCounter = 0;
    successFlag = true;
    maxFT.setZero();
    summaryFlag = false;
}

Sweetness5S0::Sweetness5S0(unsigned int fps)
{
    // Sampling period
    if (fps < 1)
    {
        // DO NOT sample anything.
        Period = 1001;
    }else
    {
        // Sample something at frequency fps.
        Period = 1000/fps;
        if (Period < 1)
        {
            Period = 1;
        }
    }
    // Time taken
    stepCounter = 0;
    logCounter = 0;
    Dura = {{0, 0, 0}};
    tTotal = 0;
    // F/T
    maxFT.setZero();
    successFlag = true;
    summaryFlag = false;
}

Sweetness5S0::~Sweetness5S0()
{
}

// Record the performance indicators. (Time only at full frequency)
void Sweetness5S0::record(unsigned int t, unsigned int p)
{
    // t: stepCounter
    // p: phaseCounter
    if (summaryFlag)
    {
        return;
    }
    if (p < 4 && p > 0)
    {
        if (t > stepCounter)
        {
            Dura[p-1]++;
            stepCounter++;
            logCounter++;
        }
    }
}

// Record the performance indicators. (Time & F/T)
void Sweetness5S0::record(unsigned int t, unsigned int p, std::array<double,6> f_ext)
{
    // t: stepCounter
    // p: phaseCounter
    // f_ext: External F/T signal
    if (summaryFlag)
    {
        return;
    }
    if (p < 4 && p > 0)
    {
        if (t > stepCounter)
        {
            Dura[p-1]++;
            stepCounter++;
            logCounter++;
            if (logCounter >= Period)
            {
                // F/T sampling in A2 or A3
                if (p > 1)
                {
                    Eigen::Map<const Eigen::Matrix<double,6,1>> Fext(f_ext.data());
                    // Only force is considered.
                    Eigen::Vector3d maxFt = maxFT.col(p-2).topLeftCorner(3,1);
                    if (maxFt.norm() < Fext.topLeftCorner(3,1).norm())
                    {
                        maxFT.col(p-2) << Fext;
                    }
                }
                logCounter = 0;
            }
        }
    }
}

// Record & save the performance indicators.
void Sweetness5S0::record(unsigned int t, unsigned int p, unsigned int s,
        std::ofstream& log_file,
        std::array<double,16> pose, std::array<double,6> f_ext)
{
    // t: stepCounter
    // p: phaseCounter
    // s: stateCounter of A3
    // log_file: file ref., [p,pose,f_ext]
    // pose: end-effector pose (SE(3))
    // f_ext: External F/T signal
    if (summaryFlag)
    {
        return;
    }
    if (p < 4 && p > 0)
    {
        if (t > stepCounter)
        {
            Dura[p-1]++;
            stepCounter++;
            logCounter++;
            if (logCounter >= Period)
            {
                log_file << p << "," << s << ",";
                // Pose data
                for (short int i = 0; i < 16; i++)
                {
                    log_file << pose[i] << ",";
                }
                // F/T data
                for (short int i = 0; i < 6; i++)
                {
                    log_file << f_ext[i] << ",";
                }
                if (p > 1)
                {
                    // A2 or A3
                    Eigen::Map<const Eigen::Matrix<double,6,1>> Fext(f_ext.data());
                    // Only force is considered.
                    Eigen::Vector3d maxFt = maxFT.col(p-2).topLeftCorner(3,1);
                    if (maxFt.norm() < Fext.topLeftCorner(3,1).norm())
                    {
                        maxFT.col(p-2) << Fext;
                    }
                }
                log_file << std::endl;
                logCounter = 0;
            }
        }
    }
}

// Record & save the performance indicators.
void Sweetness5S0::record(unsigned int t, unsigned int p, unsigned int s,
        std::ofstream& log_file,
        Eigen::Vector3d Pt, Eigen::Vector3d Pgt,
        Eigen::Quaterniond Qt, Eigen::Quaterniond Qgt,
        std::array<double,6> f_ext)
{
    // t: stepCounter
    // p: phaseCounter
    // s: stateCounter of A3
    // log_file: file ref.
    // Pt, Pgt: current position, current goal position
    // Qt, Qgt: current orientation, current goal orientation
    // f_ext: External F/T signal
    if (summaryFlag)
    {
        return;
    }
    if (p < 4 && p > 0)
    {
        if (t > stepCounter)
        {
            Dura[p-1]++;
            stepCounter++;
            logCounter++;
            if (logCounter >= Period)
            {
                log_file << p << "," << s << ",";
                // Position data
                for (short int i = 0; i < 3; i++)
                {
                    log_file << Pt(i) << ",";
                }
                for (short int i = 0; i < 3; i++)
                {
                    log_file << Pgt(i) << ",";
                }
                // Orientation data (x y z w)
                for (short int i = 0; i < 4; i++)
                {
                    log_file << Qt.coeffs()(i) << ",";
                }
                for (short int i = 0; i < 4; i++)
                {
                    log_file << Qgt.coeffs()(i) << ",";
                }
                // F/T data
                for (short int i = 0; i < 6; i++)
                {
                    log_file << f_ext[i] << ",";
                }
                if (p > 1)
                {
                    // A2 or A3
                    Eigen::Map<const Eigen::Matrix<double,6,1>> Fext(f_ext.data());
                    // Only force is considered.
                    Eigen::Vector3d maxFt = maxFT.col(p-2).topLeftCorner(3,1);
                    if (maxFt.norm() < Fext.topLeftCorner(3,1).norm())
                    {
                        maxFT.col(p-2) << Fext;
                    }
                }
                log_file << std::endl;
                logCounter = 0;
            }
        }
    }
}

// Record & save the performance indicators immediately.
void Sweetness5S0::record(unsigned int p, unsigned int s,
        std::ofstream& log_file,
        Eigen::Vector3d Pt, Eigen::Vector3d Pgt,
        Eigen::Quaterniond Qt, Eigen::Quaterniond Qgt,
        std::array<double,6> f_ext)
{
    // p: phaseCounter
    // s: stateCounter of A3
    // log_file: file ref.
    // Pt, Pgt: current position, current goal position
    // Qt, Qgt: current orientation, current goal orientation
    // f_ext: External F/T signal
    if (summaryFlag)
    {
        return;
    }
    if (p < 4 && p > 0)
    {
                log_file << p << "," << s << ",";
                // Position data
                for (short int i = 0; i < 3; i++)
                {
                    log_file << Pt(i) << ",";
                }
                for (short int i = 0; i < 3; i++)
                {
                    log_file << Pgt(i) << ",";
                }
                // Orientation data (x y z w)
                for (short int i = 0; i < 4; i++)
                {
                    log_file << Qt.coeffs()(i) << ",";
                }
                for (short int i = 0; i < 4; i++)
                {
                    log_file << Qgt.coeffs()(i) << ",";
                }
                // F/T data
                for (short int i = 0; i < 6; i++)
                {
                    log_file << f_ext[i] << ",";
                }
                if (p > 1)
                {
                    // A2 or A3
                    Eigen::Map<const Eigen::Matrix<double,6,1>> Fext(f_ext.data());
                    // Only force is considered. That's enough!
                    Eigen::Vector3d maxFt = maxFT.col(p-2).topLeftCorner(3,1);
                    if (maxFt.norm() < Fext.topLeftCorner(3,1).norm())
                    {
                        maxFT.col(p-2) << Fext;
                    }
                }
                log_file << std::endl;
    }
}

void Sweetness5S0::record(unsigned int t, unsigned int p, unsigned int s,
        std::ofstream& log_file,
        Eigen::Vector3d Pt, Eigen::Quaterniond Qt,
        std::array<double,6> f_ext, std::array<double,7> tau_c)
{
    // t: stepCounter
    // p: phaseCounter
    // s: stateCounter of A3
    // log_file: file ref.
    // Pt, Pgt: current position, current goal position
    // Qt, Qgt: current orientation, current goal orientation
    // f_ext: External F/T signal
    // tau_c: Joint torque
    if (summaryFlag)
    {
        return;
    }
    if (p < 4 && p > 0)
    {
        if (t > stepCounter)
        {
            Dura[p-1]++;
            stepCounter++;
            logCounter++;
            if (logCounter >= Period)
            {
                log_file << p << "," << s << ",";
                // Position data
                for (short int i = 0; i < 3; i++)
                {
                    log_file << Pt(i) << ",";
                }
                // Orientation data (x y z w)
                for (short int i = 0; i < 4; i++)
                {
                    log_file << Qt.coeffs()(i) << ",";
                }
                // F/T data
                for (short int i = 0; i < 6; i++)
                {
                    log_file << f_ext[i] << ",";
                }
                // Torque
                for (short int i = 0; i < 7; i++)
                {
                    log_file << tau_c[i] << ",";
                }
                if (p > 1)
                {
                    // A2 or A3
                    Eigen::Map<const Eigen::Matrix<double,6,1>> Fext(f_ext.data());
                    // Only force is considered.
                    Eigen::Vector3d maxFt = maxFT.col(p-2).topLeftCorner(3,1);
                    if (maxFt.norm() < Fext.topLeftCorner(3,1).norm())
                    {
                        maxFT.col(p-2) << Fext;
                    }
                }
                log_file << std::endl;
                logCounter = 0;
            }
        }
    }
}

// Summarize the indicators.
void Sweetness5S0::summary(bool flag)
{
    // Time taken.
    tTotal = 0;
    for (short int i = 0; i < 3; i++)
    {
        tTotal += Dura[i];
    }
    successFlag = successFlag && flag;
    summaryFlag = true;
}

// Print the evaluation results.
void Sweetness5S0::print()
{
    if (successFlag)
    {
        std::cout << "---- Results: Mission Accomplished! ----" << std::endl;
    }else
    {
        std::cout << "------- Results: Mission Failed! -------" << std::endl;
    }
    std::cout
        << "Duration of A1: " << Dura[0] << "ms" << std::endl
        << "Duration of A2: " << Dura[1] << "ms" << std::endl
        << "Duration of A3: " << Dura[2] << "ms" << std::endl
        << "Total duration: " << tTotal << "ms" << std::endl;
    if (maxFT.norm() > 0)
    {
        Eigen::Vector3d tmpFA2 = maxFT.topLeftCorner(3,1);
        Eigen::Vector3d tmpFA3 = maxFT.topRightCorner(3,1);
        Eigen::Vector3d tmpMA2 = maxFT.bottomLeftCorner(3,1);
        Eigen::Vector3d tmpMA3 = maxFT.bottomRightCorner(3,1);
        std::cout << "Max. external force in A2: " << tmpFA2.norm() << "N" << std::endl
            << "Max. external moment in A2: " << tmpMA2.norm() << "Nm" << std::endl
            << "Max. external force in A3: " << tmpFA3.norm() << "N" << std::endl
            << "Max. external moment in A3: " << tmpMA3.norm() << "Nm" << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;
}

// Print the external force. (Run record() in advance)
void Sweetness5S0::print(Eigen::Vector3d f_t, Eigen::Vector3d m_t)
{
    if (logCounter == 0)
    {
        std::cout << "f: " << f_t.transpose() << std::endl
            << "m: " << m_t.transpose() << std::endl;
    }
}

// Print one indicator
void Sweetness5S0::print(double e)
{
    if (logCounter == 0)
    {
        std::cout << e << std::endl;
    }
}

// Ticking of the counter.
bool Sweetness5S0::tick(bool seconds)
{
    if (seconds)
    {
        stepCounter++;
        logCounter++;
        if (logCounter >= Period)
        {
            logCounter = 0;
        }
    }
    return logCounter == 0;
}