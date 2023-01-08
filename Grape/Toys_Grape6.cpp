//Toys_Grape6
//  v0.0
//
//  Haopeng Hu
//  All rights reserved
//  2023.01.08
//
//  Usage: argv[0] <fci-ip> data_file_name log_file_name fps param_file_name

#include<iostream>
#include<fstream>
#include<cmath>
#include<array>
#include<vector>
#include<string>

#include<franka/robot.h>
#include<franka/model.h>
#include<franka/exception.h>
#include<franka/duration.h>

#include<Eigen/Dense>

#include"LACTIC/LACTIC.h"
#include"MILK/MILK.h"

#include"TEA/Grape6.h"

int main(int argc, char** argv){
    if (argc < 5)
    {
        std::cout << "Usage: " << argv[0] << " <fci-ip> data_file_name log_file_name FPS param_file_name" << std::endl;
        return -1;
    }
    const unsigned int FPS = std::floor(getDataFromInput(argv[4],1,1000));
    // Read the data files
    std::string data_file_name(argv[2]);
    // Init. the policy by the data file.
    Grape6 policy(data_file_name, FPS);
    policy.print();
    // Prepare the log file
    std::string log_file_name(argv[3]);
    std::ofstream log_file(log_file_name.append(".csv"),std::ios::out);
    // Exp. macros
    const int maxPeriod = 30000;
    // User message
    std::cout << "Keep the user stop at hand!" << std::endl
        << "The log data will be written in file: " << log_file_name << std::endl
        << "Press Enter to continue. Good luck!" << std::endl;
    std::cin.ignore();
    // Init. the robot.
    franka::Robot robot(argv[1]);
    franka::Model model = robot.loadModel();
    robot.setCollisionBehavior(
            {{15.0, 15.0, 12.0, 10.0, 8.0, 8.0, 8.0}}, {{15.0, 15.0, 12.0, 10.0, 8.0, 8.0, 8.0}},
            {{20.0, 20.0, 18.0, 15.0, 10.0, 10.0, 10.0}}, {{25.0, 25.0, 20.0, 18.0, 13.0, 13.0, 13.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{15.0, 15.0, 15.0, 15.0, 15.0, 15.0}}, {{45.0, 45.0, 45.0, 30.0, 30.0, 30.0}});
    std::cout << "The robot is ready to move!" << std::endl;
    // The initial pose.
    franka::RobotState s0 = robot.readOnce();
    const Eigen::Affine3d T0(Eigen::Matrix4d::Map(s0.O_T_EE.data()));
    // The external f/m bias.
    Eigen::Matrix<double,6,1> f0(Eigen::Matrix<double,6,1>::Map(s0.O_F_ext_hat_K.data()));
    // Initial impedance
    policy.A1Init(T0,{{3000, std::sqrt(1000)}}, {{200, std::sqrt(100)}});
    // Loop intermediate variable
    unsigned int stepCounter = 1;       // Time step counter
    // Impedance control law
    try
    {
        robot.control(
            [&]
            (const franka::RobotState& state, franka::Duration period)
            -> franka::Torques{
                // ************ DO NOT edit these codes ***********************
                // Robot dynamics
                std::array<double, 7> coriolis_array(model.coriolis(state));
                std::array<double, 42> jacobin_array(model.zeroJacobian(franka::Frame::kEndEffector, state));
                Eigen::Map<const Eigen::Matrix<double, 7,1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobin(jacobin_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7,1>> dqt(state.dq.data());
                // Current pose
                Eigen::Affine3d Tt(Eigen::Matrix4d::Map(state.O_T_EE.data()));
                Eigen::Vector3d Pt(Tt.translation());
                Eigen::Quaterniond Qt(Tt.linear());
                // External force
                Eigen::Map<const Eigen::Matrix<double,6,1>> f_ext(state.O_F_ext_hat_K.data());
                Eigen::Matrix<double,6,1> f_ext1(f_ext - f0);
                // Pose error
                Eigen::VectorXd Et(6);
                Et.setZero();
                Et.head(3) << policy.cnt.Pgt - Pt;
                Et.tail(3) << quatSubtraction(policy.cnt.Qgt, Qt);
                // Control law
                Eigen::VectorXd tau(7);
                std::array<double,7> tau_c_array;
                tau << jacobin.transpose() * (policy.cnt.K * Et - policy.cnt.D * (jacobin * dqt)) + coriolis;
                Eigen::VectorXd::Map(&tau_c_array[0],7) = tau;
                franka::Torques tau_c(tau_c_array);
                // *************************************************************
                // **** A3 Policy ****
                bool terminalFlag = false;
                if (policy.getCurrPhase()==1)
                {
                    if (policy.A1Terminal(Pt,Qt,{{0.0005, 0.1}}))
                    {
                        // Init. A2
                        policy.A2Init(Pt,Qt,800,{{20,0.001*std::sqrt(380)}},
                            200,{{10,0.001*std::sqrt(200)}});
                        // F/M bias
                        f0 << Eigen::Matrix<double,6,1>::Map(state.O_F_ext_hat_K.data());
                    }else
                    {
                        policy.A1GoalUpdate(10,Pt,{{0.015, 1.0}},50); // Independent interpolation
                        // Convergence margin
                        terminalFlag = policy.A1BeyondMargin(Pt,Qt,{{0.010, 0.2}});
                    }
                }else if (policy.getCurrPhase()==2)
                {
                    if (policy.A2Terminal(Pt,0.0001))
                    {
                        policy.A3Init(Pt,Qt,{{600, std::sqrt(600)}},{{50, std::sqrt(50)}},
                            {{100, std::sqrt(10)}}, {{1.0, std::sqrt(1.0)}});
                        // F/M bias
                        f0 << Eigen::Matrix<double,6,1>::Map(state.O_F_ext_hat_K.data());
                    }else
                    {
                        policy.A2GoalUpdate(f_ext1, 0.04, {{0.05, 0.01}});
                        // Convergence margin
                        terminalFlag = policy.A2BeyondMargin(Pt,Qt,{{0.010, 0.4}});
                    }
                }else if (policy.getCurrPhase()==3)
                {
                    if (policy.A3Terminal(Pt,Qt,f_ext,{{0.0032, 0.020}},{{25, 16}}))
                    {
                        terminalFlag = true;
                    }else
                    {
                        policy.A3GoalUpdate(f_ext1,{{0.05, 0.40}},{{0.025,0.020}});
                    }
                }
                // **** Log data ****
                policy.pfm.record(stepCounter,policy.getCurrPhase(),policy.getCurrState(),
                    log_file,
                    Pt,policy.cnt.Pgt,Qt,policy.cnt.Qgt,state.O_F_ext_hat_K);
                /*
                if (policy.getCurrState() == 2)
                {
                    //Eigen::Vector3d e_w(Redgrape10::quatSubstract(policy.q_s[1],Qt,false));
                    //policy.pfm.print(e_w.dot(policy.d_s[1]));
                    //policy.pfm.print(e_w.norm());
                    policy.pfm.print(f_ext.head(3),f_ext1.head(3));
                }*/
                stepCounter ++;
                // **** Timeout or Terminal condition ****
                if (stepCounter > maxPeriod || terminalFlag)
                {
                    //tau_c.tau_J = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                    policy.pfm.record(policy.getCurrPhase(),policy.getCurrState(),
                        log_file,
                        Pt,policy.cnt.Pgt,Qt,policy.cnt.Qgt,state.O_F_ext_hat_K);
                    std::cout << "Terminal mod: " << policy.terminalMod << std::endl
                        << "Current SAMP state: " << policy.getCurrState() << std::endl;
                    return franka::MotionFinished(tau_c);
                }
                /*
                if (policy.getCurrPhase() > 3)
                {
                    tau_c.tau_J = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                    return franka::MotionFinished(tau_c);
                }*/
                return tau_c;
            }
        );
    }
    catch(const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        log_file.close();
        policy.pfm.summary(false);
        policy.pfm.print();
        return -1;
    }
    policy.summary();;
    policy.pfm.print();
    std::cout << "Experiment done!" << std::endl;
    log_file.close();
    return 0;
}

