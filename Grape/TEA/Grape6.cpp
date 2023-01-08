//Grape6

#include "Grape6.h"

Grape6::Grape6()
{
    this->defaultBehavior();
}

// Init. it with the model file. v5S.1.1
Grape6::Grape6(std::string file_name, unsigned int FPS)
{
    this->defaultBehavior();
    // Model params. read
    std::vector<std::vector<double>> ExpData = this->readCSV(file_name);
    // A1 & A2
    Eigen::Map<Eigen::Vector3d> p_l(&ExpData[0][0]);
    Eigen::Map<Eigen::Quaterniond> q_s0(&ExpData[0][3]);
    Eigen::Map<Eigen::Vector3d> d_l(&ExpData[0][7]);
    Eigen::Map<Eigen::Vector3d> p_s0(&ExpData[1][0]);
    this->p_l = p_l;
    this->q_s0 = q_s0.normalized();
    this->d_l = d_l.normalized();
    this->p_s0 = p_s0;
    // A3
    ns = ExpData.size() - 2;
    for (unsigned int i = 0; i < ns; i++)
    {
        Eigen::Map<Eigen::Vector3d> p_s(&ExpData[2+i][1]);
        Eigen::Map<Eigen::Quaterniond> q_s(&ExpData[2+i][4]);
        Eigen::Map<Eigen::Vector3d> d_s(&ExpData[2+i][8]);
        this->mod[i] = ExpData[2+i][0];
        this->p_s[i] = p_s;
        this->q_s[i] = q_s.normalized();
        this->d_s[i] = d_s.normalized();
    }
    this->pfm = Sweetness5S0(FPS);
}

// Read the .csv file
std::vector<std::vector<double>> Grape6::readCSV(std::string fileName)
{
    fileName.append(".csv");
    std::vector<std::vector<double>> dataRead;
    std::ifstream fileIn(fileName,std::ios::in);
    if(fileIn.fail()){
        std::cerr << "File: " << fileName << " not found" << std::endl;
        throw std::exception();
        return dataRead;
    }
    std::string line;
    while (std::getline(fileIn,line) && fileIn.good())
    {
        std::istringstream dataIn(line);
        std::string dataTmp;
        std::vector<double> dataReadTmp;
        while (std::getline(dataIn,dataTmp,','))
        {
            dataReadTmp.push_back(std::stod(dataTmp));
        }
        dataRead.push_back(dataReadTmp);
    }
    std::cout << dataRead.size() << " data in " << fileName << " have been read." << std::endl;
    return dataRead;
}

// Print the model params.
void Grape6::print()
{
    std::cout << "-------------- Policy --------------" << std::endl;
    std::cout << "p_l: " << p_l.transpose() << std::endl
        << "p_s0: " << p_s0.transpose() << std::endl
        << "q_s0: " << q_s0.coeffs().transpose() << std::endl
        << "d_l: " << d_l.transpose() << std::endl;
    std::cout << "SAMP: " << std::endl;
    for (short int i = 0; i < ns; i++)
    {
        std::cout << "AMP Mode: " << mod[i] << std::endl
            << "p_s: " << p_s[i].transpose() << std::endl
            << "q_s: " << q_s[i].coeffs().transpose() << std::endl
            << "d_s: " << d_s[i].transpose() << std::endl;
    }
    std::cout << "-----------------------------------" << std::endl;
}

// Summary the results
void Grape6::summary()
{
    if (this->p < 3)
    {
        // A1 or A2
        pfm.summary(this->terminalMod == 0);
    }else
    {
        // A3
        pfm.summary(this->terminalMod < 3);
    }
}

// Get the ID of current phase
unsigned int Grape6::getCurrPhase()
{
    return this->p;
}

// Get the ID of current state in A3.
unsigned int Grape6::getCurrState(bool typeFlag)
{
    if (this->p >= 3)
    {
        if (typeFlag)
        {
            // Return the type ID 
            return mod[ts];
        }else
        {
            // Return the sequence ID
            return ts+1;
        }
    }else
    {
        return 0;
    }
}

// Calculate the desired frame
void Grape6::desiredFrameR(Eigen::Vector3d d)
{
    Eigen::Vector3d x(1,0,0);
    double e = std::sqrt(x.dot(d))/d.norm();
    if (1-e < 0.001)
    {
        Rsd = Eigen::MatrixXd::Identity(3,3);
    }else
    {
        Eigen::Vector3d yd = x.cross(d); yd.normalize();
        Eigen::Vector3d zd = d.cross(yd); zd.normalize();
        Rsd.setZero();
        Rsd.col(0) << d;
        Rsd.col(1) << yd;
        Rsd.col(2) << zd;
    }
}

// Calculate the desired impedance
void Grape6::desiredImpedance(unsigned int mod,
    std::array<double,2> kd, std::array<double,2> kc,
    std::array<double,2> dd, std::array<double,2> dc)
{
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);
    if (mod == 1)
    {
        // Translation
        K.topLeftCorner(3,3) << setDiagMatrix(kd[0],kc[0]);
        K.bottomRightCorner(3,3) << setDiagMatrix(kc[1], kc[1]);
        D.topLeftCorner(3,3) << setDiagMatrix(dd[0],dc[0]);
        D.bottomRightCorner(3,3) << setDiagMatrix(dc[1], dc[1]);
    }else if (mod == 2)
    {
        // Rotation
        K.topLeftCorner(3,3) << setDiagMatrix(kc[0],0.0);
        K.bottomRightCorner(3,3) << setDiagMatrix(kd[1], kc[1]);
        D.topLeftCorner(3,3) << setDiagMatrix(dc[0],0.0);
        D.bottomRightCorner(3,3) << setDiagMatrix(dd[1], dc[1]);
    }
    cnt.K = K;
    cnt.D = D;
    cnt.K.topLeftCorner(3,3) << Rsd * K.topLeftCorner(3,3) * Rsd.transpose();
    cnt.D.topLeftCorner(3,3) << Rsd * D.topLeftCorner(3,3) * Rsd.transpose();
}

// Set the diagonal SPD matrix
Eigen::Matrix3d Grape6::setDiagMatrix(double k_x, double k_yz)
{
    Eigen::Matrix3d M = k_yz * Eigen::MatrixXd::Identity(3,3);
    M(0,0) = k_x;
    return M;
}

// Set the default parameters and variables
void Grape6::defaultBehavior()
{
    this->Rsd.setIdentity();
    this->t = 1;
    this->p = 1;
    this->ts = 0;
    this->terminalMod = 0;
    this->f_thd = 10;
    this->m_thd = 1.0;
    this->f_max = 25;
    this->kd = {{0.0, 0.0}};
    this->kc = {{0.0, 0.0}};
    this->dd = {{0.0, 0.0}};
    this->dc = {{0.0, 0.0}};
}

Grape6::~Grape6()
{
}

// A3 and SAMP

// Init. the Approaching AMP
void Grape6::A1Init(Eigen::Affine3d T0, std::array<double,2> kd, 
    std::array<double,2> dd)
{
    // T0, the current pose.
    // kd, desired stiffness of p/q;
    // dd, desired damping of p/q;
    cnt.Pgt = T0.translation();
    cnt.Qgt = T0.linear();
    // Desired frame
    desiredFrameR(d_l);
    // Desired impedance
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);
    K.topLeftCorner(3,3) << setDiagMatrix(kd[0],kd[0]);
    K.bottomRightCorner(3,3) << setDiagMatrix(kd[1], kd[1]);
    D.topLeftCorner(3,3) << setDiagMatrix(dd[0],dd[0]);
    D.bottomRightCorner(3,3) << setDiagMatrix(dd[1], dd[1]);
    // Spatial imp.
    cnt.K = K;
    cnt.D = D;
    this->p = 1;
    this->t = 1;
}

// Update the goal pose in the approaching phase
void Grape6::A1GoalUpdate(unsigned int PERIOD, Eigen::Vector3d pt,
    std::array<double,2> coef, double scal)
{
    // PERIOD: the interpolation period. Set it zero to use adaptive period
    // coef[0]: the goal position interpolation coef.
    // coef[1]: the goal orientation interpolation coef.
    // scal: the scalor of vertical direction updating step prop. to parallel one.
    if (PERIOD == 0)
    {
        // Adaptive interpolation period
        // ????
    }else
    {
        // Constant interpolation period
        if (this->t < PERIOD)
        {
            this->t++;
            return;
        }
    }
    // Position update
    Eigen::Vector3d et(p_l - cnt.Pgt);
    Eigen::Vector3d ev(et - et.dot(d_l)*d_l);
    cnt.Pgt << cnt.Pgt + 0.01 * (coef[0] * d_l + scal*coef[0]*ev);
    // Orientation update
    Eigen::Vector3d eta(cnt.quatSubstract(q_s0,cnt.Qgt,false));
    Eigen::Quaterniond dq(cnt.quatExpMap(((0.01*coef[1])*eta)));
    cnt.Qgt = cnt.quatProduct(dq,cnt.Qgt);
    this->t = 1;
}

// Terminal the A1 phase.
bool Grape6::A1Terminal(Eigen::Vector3d pt, Eigen::Quaterniond qt,
    std::array<double,2> maxErr)
{
    // maxErr[0]: position error allowed
    // maxErr[1]: orientation error (rad) allowed
    bool flag = false;
    Eigen::Vector3d Etat = cnt.quatSubstract(q_s0,qt,false);
    double eQt = 2*Etat.norm();
    if (eQt <= maxErr[1])
    {
        // Orientation of EE is regulated!
        // Considering the position of EE
        Eigen::Vector3d EPt = p_l - pt;
        double ePt = EPt.dot(d_l);
        flag = ePt < maxErr[0];
    }else
    {
        flag = false;
    }
    return flag;
}

// Detecting whether the pose is beyond the convergence margin.
bool Grape6::A1BeyondMargin(Eigen::Vector3d pt, Eigen::Quaterniond qt, std::array<double,2> margin)
{
    // margin[0]: the position margin
    // margin[1]: the orientation margin (rad)
    bool flag = false;
    Eigen::Vector3d e(p_l - pt);
    if (e.dot(d_l) < 0.00005) // It arrived at the desired depth
    {
        // Position margin
        Eigen::Vector3d ev = e - e.dot(d_l)*d_l;
        flag = flag || (ev.norm() > margin[0]);
        // Orientation margin
        flag =  flag || cnt.quatError(q_s0,qt) > margin[1];
    }
    if (flag)
    {   // Beyond the convergence margin
        terminalMod = 3;
    }
    return flag;
}

// Init. the aligning phase
void Grape6::A2Init(Eigen::Vector3d p_t, Eigen::Quaterniond q_t,
    double kd_p, std::array<double,2> kc, double dd_p, std::array<double,2> dc)
{
    // p_t: current position
    // q_t: current orientation
    // kd_p: desired stiffness of position
    // kc: compliant stiffness of position/orientation
    // dd: desired damping of position
    // dc: compliant damping of position/orientation
    cnt.Pgt = p_t;
    cnt.Qgt = q_t;
    // Desired frame
    desiredFrameR(d_l);
    // Desired impedance
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);
    K.topLeftCorner(3,3) << setDiagMatrix(kd_p,kc[0]);
    K.bottomRightCorner(3,3) << setDiagMatrix(kc[1], kc[1]);
    D.topLeftCorner(3,3) << setDiagMatrix(dd_p,dc[0]);
    D.bottomRightCorner(3,3) << setDiagMatrix(dc[1], dc[1]);
    // Spatial imp.
    cnt.K = K;
    cnt.D = D;
    cnt.K.topLeftCorner(3,3) << Rsd * K.topLeftCorner(3,3) * Rsd.transpose();
    cnt.D.topLeftCorner(3,3) << Rsd * D.topLeftCorner(3,3) * Rsd.transpose();
    this->p = 2;
}

// Update the goal pose in the aligning phase given F/T feedback.
void Grape6::A2GoalUpdate(Eigen::Matrix<double,6,1> f_ext, double step, std::array<double,2> coefs)
{
    // f_ext: the external force
    // step: For the desired direction.
    // coefs[0]: For the compliant translation directions.
    // coefs[1]: For the orientation aligning. 

    // Calculate the f_ext along the compliant directions.
    Eigen::Vector3d fd = -1 * Rsd.transpose() * f_ext.head(3);
    Eigen::Vector3d md = -1 * Rsd.transpose() * f_ext.tail(3);
    double coef_des = 1;
    Eigen::Matrix3d extrTran_cmp_p = Eigen::MatrixXd::Zero(3,3);
    Eigen::Matrix3d extrTran_cmp_q = Eigen::MatrixXd::Zero(3,3);
    if (fd(0) >= f_thd)
    {
        extrTran_cmp_p = Eigen::MatrixXd::Identity(3,3);
        extrTran_cmp_p(0,0) = 0;
        extrTran_cmp_q = Eigen::MatrixXd::Identity(3,3);
        if (std::abs(fd(0)) >= f_max)
        {
            coef_des = 0;
        }
    }
    // Position update.
    cnt.Pgt << cnt.Pgt + 0.001 * (step * coef_des * d_l 
        + coefs[0] * Rsd * extrTran_cmp_p * fd);
    // Orientation update.
    cnt.Qgt = cnt.quatUpdate(cnt.Qgt,coefs[1]* Rsd * extrTran_cmp_q * md,0.001);
}

// Detect whether the aligning phase is over.
bool Grape6::A2Terminal(Eigen::Vector3d pt, double maxErr)
{
    Eigen::Vector3d Et(p_s0 - pt);
    bool flag = Et.dot(d_l) <= maxErr;
    return flag;
}

// Detecting whether the pose is beyond the convergence margin.
bool Grape6::A2BeyondMargin(Eigen::Vector3d pt, Eigen::Quaterniond qt, std::array<double,2> margin)
{
    // margin[0]: the position margin
    // margin[1]: the orientation margin (rad)
    bool flag = false;
    Eigen::Vector3d e(p_s0 - pt);
    // Position margin
    Eigen::Vector3d ev = e - e.dot(d_l)*d_l;
    flag = flag || (ev.norm() > margin[0]);
    // Orientation margin
    flag =  flag || cnt.quatError(q_s0,qt) > margin[1];
    if (flag)
    {   // Beyond the convergence margin
        terminalMod = 3;
    }
    return flag;
}

// Init. the assembling phase
void Grape6::A3Init(Eigen::Vector3d pt, Eigen::Quaterniond qt,
        std::array<double,2> kd, std::array<double,2> kc,
        std::array<double,2> dd, std::array<double,2> dc)
{
    // p_t: current position
    // q_t: current orientation
    // kd: desired stiffness of position/orientation
    // kc: compliant stiffness of position/orientation
    // dd: desired damping of position/orientation
    // dc: compliant damping of position/orientation
    cnt.Pgt = pt;
    cnt.Qgt = qt;
    this->kd = kd;
    this->kc = kc;
    this->dd = dd;
    this->dc = dc;
    this->f_thd = 2.0;
    this->m_thd = 1.0;

    this->p = 3;
    this->ts = 0;
    // The first AMP
    desiredFrameR(d_s[ts]);
    desiredImpedance(mod[ts],kd,kc,dd,dc);
}

// Update the goal pose in the assembling phase
void Grape6::A3GoalUpdate(Eigen::Matrix<double,6,1> f_ext, std::array<double,2> step,
        std::array<double,2> coefs)
{
    // step[0]: goal position update step.
    // step[1]: goal orientation update step.
    // coefs[0]: position admittance params.
    // coefs[1]: orientation admittance params.

    Eigen::Vector3d fd = -1 * Rsd.transpose() * f_ext.head(3);
    Eigen::Vector3d md = -1 * Rsd.transpose() * f_ext.tail(3);

    Eigen::Matrix3d extrTran_cmp_p = Eigen::MatrixXd::Zero(3,3);
    Eigen::Matrix3d extrTran_cmp_q = Eigen::MatrixXd::Zero(3,3);
    if (mod[ts] == 1)
    {
        // Translation
        if (fd.tail(2).norm() >= f_thd)
        {
            extrTran_cmp_p = Eigen::MatrixXd::Identity(3,3);
            extrTran_cmp_p(0,0) = 0;
        }
        if (md.norm() >= m_thd)
        {
            extrTran_cmp_q = Eigen::MatrixXd::Identity(3,3);
        }
        // Position update.
        cnt.Pgt << cnt.Pgt + 0.001 * (step[0] * d_s[ts] 
            + coefs[0] * Rsd * extrTran_cmp_p * fd);
        // Orientation update.
        cnt.Qgt = cnt.quatUpdate(cnt.Qgt,coefs[1]* Rsd * extrTran_cmp_q * md,0.001);
    }else if (mod[ts] == 2)
    {
        // Rotation
        if (fd(0) >= f_thd)
        {
            extrTran_cmp_p(0,0) = 1;
        }
        if (md.tail(2).norm() >= m_thd)
        {
            extrTran_cmp_q = Eigen::MatrixXd::Identity(3,3);
            extrTran_cmp_q(0,0) = 0;
        }
        // Position update.
        cnt.Pgt << cnt.Pgt + 0.001 * (coefs[0] * Rsd * extrTran_cmp_p * fd);
        // Orientation update.
        cnt.Qgt = cnt.quatUpdate(cnt.Qgt, step[1]* d_s[ts] + coefs[1] * Rsd * extrTran_cmp_q * md, 0.001);
    }
}

// Termine or switch the SAMP states in the assembling phase.
bool Grape6::A3Terminal(Eigen::Vector3d pt, Eigen::Quaterniond qt, Eigen::Matrix<double,6,1> f_ext,
        std::array<double,2> maxErr, std::array<double,2> maxFM)
{
    // maxErr[0]: max. position error allowed
    // maxErr[1]: max. orientation error allowed (rad)
    // maxFM[0]: max. external force allowed
    // maxFM[1]: max. external moment allowed

    bool flag = false;
    Eigen::Vector3d fd = Rsd.transpose() * f_ext.head(3);
    Eigen::Vector3d md = Rsd.transpose() * f_ext.tail(3);
    if (fd.norm() >= f_max)
    {
        // Safe notification
        flag = true;
        terminalMod = 4;
        return flag;
    }
    if (mod[ts] == 1)
    {
        // Translation
        Eigen::Vector3d e(p_s[ts]-pt);
        if (fd(0) >= maxFM[0] && e.norm() < 10 * maxErr[0])
        {
            // Forceout
            flag = true;
            terminalMod = 1;
        }else
        {
            flag = e.dot(d_s[ts]) <= maxErr[0];
        }
    }else if (mod[ts] == 2)
    {
        // Rotation
        Eigen::Vector3d e(Redgrape10::quatSubstract(q_s[ts],qt,false));
        if (md(0) >= maxFM[1] && e.norm() < 10 * maxErr[1])
        {
            // Momentout
            flag = true;
            terminalMod = 2;
        }else
        {
            flag = e.dot(d_s[ts]) <= maxErr[1];
        }
    }
    // Switch
    if (flag && (ts < ns -1))
    {
        // Switch!
        flag = false;
        ts++;
        // Spatial controller
        desiredFrameR(d_s[ts]);
        desiredImpedance(mod[ts],kd,kc,dd,dc);
        //std::cout << "Switch to " << ts << std::endl;
    }
    return flag;
}