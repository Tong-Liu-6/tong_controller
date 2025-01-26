#include "MPC_diffDrive_fblin.h"

#include <cmath>
#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>

MPC_diffDrive_fblin::MPC_diffDrive_fblin() {
    // Initialize class variables
    _robotParamsInitialized = false;
    _MPCparamsInitialized = false;
    _FBLINparamsInitialized = false;
    _controllerInitialized = false;

    _linearVelocity = _angularVelocity = 0.0;

    // Initialize pointers
    _solver = NULL;
    _fblinController = _fblinSimController = NULL;
    _debug = _info = _error = NULL;
}

MPC_diffDrive_fblin::~MPC_diffDrive_fblin() {
    // Destroy solver object
    if (_solver)
    {
        delete _solver;
        _solver = NULL;
    }

    // Destroy linearization object
    if (_fblinController)
    {
        delete _fblinController;
        _fblinController = NULL;
    }
    if (_fblinSimController)
    {
        delete _fblinSimController;
        _fblinSimController = NULL;
    }
}

void MPC_diffDrive_fblin::set_MPCparams(double samplingTime, int predictionHorizon, double q, double r) {
    std::vector<double> lb(2, -INFINITY);
    std::vector<double> ub(2, +INFINITY);

    // Set MPC params
    set_MPCparams(samplingTime, predictionHorizon, q, r, lb, ub);
}

void MPC_diffDrive_fblin::set_MPCparams(double samplingTime, int predictionHorizon, double q, double r,
                                        const std::vector<double>& variableLB, const std::vector<double>& variableUB) {
    // Set MPC parameters
    _q = q;
    _r = r;
    _MPC_Ts = samplingTime;
    _N = predictionHorizon;

    // Set variable range
    _lowerBound.reserve(2*_N);
    _upperBound.reserve(2*_N);
    for (auto i=0; i<_N; i++) {
        _lowerBound.insert(_lowerBound.end(), variableLB.begin(), variableLB.end());
        _upperBound.insert(_upperBound.end(), variableUB.begin(), variableUB.end());
    }

    // Set the initialization flag
    _MPCparamsInitialized = true;
}

void MPC_diffDrive_fblin::set_FBLINparams(double samplingTime, double pointPdistance) {
    // Set feedback linearization parameters
    _fblin_Ts = samplingTime;
    _Pdist = pointPdistance;

    // Set the initialization flag
    _FBLINparamsInitialized = true;
}

void MPC_diffDrive_fblin::set_robotParams(double wheelVelMax, double wheelVelMin, double wheelRadius, double track) {
    // Set robot parameters
    _wheelVelMax = wheelVelMax;
    _wheelVelMin = wheelVelMin;
    _wheelRadius = wheelRadius;
    _track = track;

    // Set the initialization flag
    _robotParamsInitialized = true;
}

bool MPC_diffDrive_fblin::initialize() {
    /** Preliminary checks */
    if (!_robotParamsInitialized)
    {
        errorMsg("[MPC_diffDrive_fblin.initialize] Call set_robotParams() before calling initialize()");
        return false;
    }
    if (!_FBLINparamsInitialized)
    {
        errorMsg("[MPC_diffDrive_fblin.initialize] Call set_FBLINparams() before calling initialize()");
        return false;
    }
    if (!_MPCparamsInitialized)
    {
        errorMsg("[MPC_diffDrive_fblin.initialize] Call set_MPCparams() before calling initialize()");
        return false;
    }
    if (std::remainder(_MPC_Ts, _fblin_Ts)>=1.0e-15)
    {
        errorMsg("[MPC_diffDrive_fblin.initialize] MPC and feedback linearization sampling times should be multiple");
        return false;
    }

    // Initialize plant matrices
    _plant_A = Eigen::MatrixXd::Identity(2,2);
    _plant_B = _MPC_Ts*Eigen::MatrixXd::Identity(2,2);

    // Initialize MPC controller parameters
    _k = -1.0/(2.0*_MPC_Ts);
    _p = (_q+pow(_k, 2.0)*_r)/(1.0-pow(1.0+_MPC_Ts*_k, 2.0));

    // Initialize MPC controller matrices
    compute_AcalMatrix();
    compute_BcalMatrix();
    compute_QcalMatrix();
    compute_RcalMatrix();

    _H = Eigen::MatrixXd::Zero(2*_N, 2*_N);
    _f = Eigen::VectorXd::Zero(2*_N);
    _Ain_vel = Eigen::MatrixXd::Zero(2*2*_N, 2*_N);
    _Bin_vel = Eigen::VectorXd::Zero(2*2*_N);

    // Initialize actual and reference data
    _actX = _actY = _actYaw = 0.0;
    _actXP = _actYP = 0.0;
    _predictRobotState = Eigen::VectorXd::Zero(3*(_N+1));
    _refMPCstate = Eigen::VectorXd::Zero(2*(_N+1));
    _optimVect = Eigen::VectorXd::Zero(2*_N);

    // Initialize the solver
    _solver = new GUROBIsolver(GUROBI_LICENSEID, GUROBI_USERNAME);
    if (!_solver->initProblem(2*_N, _lowerBound, _upperBound))
    {
        errorMsg("[MPC_diffDrive_fblin.initialize] Error initializing the solver");
        return false;
    }

    // Initialize the linearization controller
    _fblinController = new fblin_unicycle(_Pdist);
    _fblinSimController = new fblin_unicycle(_Pdist);

    // Set the initialization flag
    _controllerInitialized = true;

    return true;
}

bool MPC_diffDrive_fblin::executeMPCcontroller() {
    /** Preliminary checks */
    if (!_controllerInitialized)
    {
        errorMsg("[MPC_diffDrive_fblin.executeMPCcontroller] Call initialize() before calling executeMPCcontroller()");
        return false;
    }

    // Compute the prediction of the plant states based on the previous step control vector
    double act_x, act_y, act_theta;
    _predictRobotState(0) = act_x = _actX;
    _predictRobotState(1) = act_y = _actY;
    _predictRobotState(2) = act_theta = _actYaw;

    for (auto k=0; k<_N; k++) {
        for (auto i=0; i<_MPC_Ts/_fblin_Ts; i++) {
            // Update the linearizing controller state
            _fblinSimController->set_unicycleState(act_x, act_y, act_theta);

            // Compute the robot velocities
            double v, w;
            _fblinSimController->control_transformation(_optimVect(2*k), _optimVect(2*k+1), v, w);

            // Compute the next robot state
            act_x += v*_fblin_Ts*std::cos(act_theta+w*_fblin_Ts/2.0);
            act_y += v*_fblin_Ts*std::sin(act_theta+w*_fblin_Ts/2.0) ;
            act_theta += w*_fblin_Ts;
        }

        // Store the computed robot state
        _predictRobotState(3*(k+1)) = act_x;
        _predictRobotState(3*(k+1)+1) = act_y;
        _predictRobotState(3*(k+1)+2) = act_theta;
    }

    // Compute constraint matrices
    compute_wheelVelocityConstraint();
    if (wheelVelocityConstrain.size()==0) {
        if (!_solver->addConstraint(_Ain_vel, _Bin_vel, wheelVelocityConstrain))
        {
            errorMsg("[MPC_diffDrive_fblin.executeMPCcontroller] Error setting the wheel velocity constraint");
            return false;
        }
    }
    else {
        if (!_solver->modifyConstraint(_Ain_vel, _Bin_vel, wheelVelocityConstrain))
        {
            errorMsg("[MPC_diffDrive_fblin.executeMPCcontroller] Error setting the wheel velocity constraint");
            return false;
        }
    }

    // Compute cost function matrices
    compute_objectiveMatrix();
    if (!_solver->setObjective(_H, _f))
    {
        errorMsg("[MPC_diffDrive_fblin.executeMPCcontroller] Error setting the MPC objective");
        return false;
    }

    // Solve optimization problem
    int optimizerStatus;
    double objectiveValue;
    if (!_solver->solveProblem(_optimVect, objectiveValue, optimizerStatus))
    {
        _optimVect.setZero();

        switch (optimizerStatus) {
            case GUROBIsolver::INFEASIBLE:
                errorMsg("[MPC_diffDrive_fblin.executeMPCcontroller] Error solving the optimization problem (infeasible problem)");
                break;
            case GUROBIsolver::OTHER:
                errorMsg("[MPC_diffDrive_fblin.executeMPCcontroller] Error solving the optimization problem (not optimal but not infeasible)");
                break;
        }
        return false;
    }

    return true;
}

bool MPC_diffDrive_fblin::executeLinearizationController() {
    /** Preliminary checks */
    if (!_controllerInitialized) {
        errorMsg("[MPC_diffDrive_fblin.executeLinearizationController] Call initialize() before calling executeLinearizationController()");
        return false;
    }

    // Execute the feedback linearization law
    _fblinController->control_transformation(_optimVect(0), _optimVect(1), _linearVelocity, _angularVelocity);

    return true;
}

void MPC_diffDrive_fblin::set_actualRobotState(double x, double y, double yaw) {
    _actX = x;
    _actY = y;
    _actYaw = yaw;

    // Update the linearizing controller state
    _fblinController->set_unicycleState(_actX, _actY, _actYaw);

    // Update the MPC state
    _fblinController->ouput_transformation(_actXP, _actYP);

}

void MPC_diffDrive_fblin::set_referenceRobotState(double x, double y, double yaw)
{
    // Transform from robot state to point P state
    double xP_ref, yP_ref;
    _fblinController->reference_transformation(x, y, yaw, xP_ref, yP_ref);

    // Fill in the reference vector along the prediction horizon with a constant reference
    for (auto i=0; i<_N+1; i++) {
        _refMPCstate.block(i*2, 0, 2, 1) = Eigen::Vector2d(xP_ref, yP_ref);
    }
}

void MPC_diffDrive_fblin::set_referenceRobotState(const Eigen::VectorXd& refRobotState)
{
    if (refRobotState.size()!=3*(_N+1))
    {
        errorMsg("[MPC_diffDrive_fblin.set_referenceRobotState] The refState variable has the wrong size");
        _refMPCstate = refRobotState.segment(0, 3*(_N+1));
    }
    else
    {
        // Transform from robot state to point P state and store in reference vector
        for (auto i=0; i<_N+1; i++) {
            _fblinController->reference_transformation(refRobotState(3*i), refRobotState(3*i+1),
                                                       refRobotState(3*i+2), _refMPCstate(2*i), _refMPCstate(2*i+1));
        }
    }
}

void MPC_diffDrive_fblin::get_actualMPCControl(double& vPx, double& vPy)
{
    vPx = _optimVect(0);
    vPy = _optimVect(1);
}

void MPC_diffDrive_fblin::get_actualMPCstate(double& xP, double& yP)
{
    xP = _actXP;
    yP = _actYP;
}

void MPC_diffDrive_fblin::get_referenceMPCstate(Eigen::VectorXd& refState)
{
    refState = _refMPCstate;
}

void MPC_diffDrive_fblin::get_actualControl(double &linVelocity, double &angVelocity)
{
    linVelocity = _linearVelocity;
    angVelocity = _angularVelocity;
}


/** Private methods */
void MPC_diffDrive_fblin::compute_AcalMatrix() {
    // Initialize Acal matrix
    _Acal = Eigen::MatrixXd::Zero((_N+1)*2, 2);

    // Compute Acal matrix
    _Acal.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    for (int k=0; k<_N+1; k++) {
        _Acal.block(2*k, 0, 2, 2) = _plant_A.pow(k);
    }

    // Check matrix
//    saveMatrixToFile("Acal_matrix.csv", _Acal);
}

void MPC_diffDrive_fblin::compute_BcalMatrix() {
    // Initialize Bcal matrix
    _Bcal = Eigen::MatrixXd::Zero((_N+1)*2, _N*2);

    // Compute Bcal matrix
    for (int i=1; i<=_N; i++)
        for (int j=1; j<=i; j++)
            _Bcal.block(2*i, 2*(j-1), 2, 2) = _plant_A.pow(i-j)*_plant_B;

    // Check matrix
//    saveMatrixToFile("Bcal_matrix.csv", _Bcal);
}

void MPC_diffDrive_fblin::compute_QcalMatrix() {
    // Initialize Qcal matrix
    _Qcal = Eigen::MatrixXd::Zero((_N+1)*2, (_N+1)*2);

    // Compute Qcal matrix
    _Qcal.diagonal() << Eigen::VectorXd::Ones(_N*2)*_q, Eigen::VectorXd::Ones(2)*_p;

    // Check matrix
//    saveMatrixToFile("Qcal_matrix.csv", _Qcal);
}

void MPC_diffDrive_fblin::compute_RcalMatrix() {
    // Initialize Rcal matrix
    _Rcal = Eigen::MatrixXd::Zero(_N*2, _N*2);

    // Compute Rcal matrix
    _Rcal.diagonal() = Eigen::VectorXd::Ones(_N*2)*_r;

    // Check matrix
//    saveMatrixToFile("Rcal_matrix.csv", _Rcal);
}

void MPC_diffDrive_fblin::compute_objectiveMatrix() {
    // Initialize H and f matrices
    _H = Eigen::MatrixXd::Zero(2*_N, 2*_N);
    _f = Eigen::VectorXd::Zero(_H.rows());

    // Compute H and f matrices
    _H = _Bcal.transpose()*_Qcal*_Bcal+_Rcal;
    _f = (_Acal*Eigen::Vector2d(_actXP, _actYP)-_refMPCstate).transpose()*_Qcal*_Bcal;

    // Check matrix
//    saveMatrixToFile("H_matrix.csv", _H);
//    saveMatrixToFile("f_matrix.csv", _f);
}

void MPC_diffDrive_fblin::compute_wheelVelocityConstraint() {
    // Initialize Ain_vel and Bin_vel matrices
    _Ain_vel = Eigen::MatrixXd::Zero(2*2*_N, 2*_N);
    _Bin_vel = Eigen::VectorXd::Zero(2*2*_N);

    // Compute constraint matrices
    for (auto k=0; k<_N; k++) {
        Eigen::Matrix2d Abar {{2.0*std::cos(_predictRobotState(3*k+2))-_track/_Pdist*std::sin(_predictRobotState(3*k+2)),
                               2.0*std::sin(_predictRobotState(3*k+2))+_track/_Pdist*std::cos(_predictRobotState(3*k+2))},
                              {2.0*std::cos(_predictRobotState(3*k+2))+_track/_Pdist*std::sin(_predictRobotState(3*k+2)),
                               2.0*std::sin(_predictRobotState(3*k+2))-_track/_Pdist*std::cos(_predictRobotState(3*k+2))}};
        _Ain_vel.block(2*k, 2*k, 2, 2) = Abar;
        _Ain_vel.block(2*(k+_N), 2*k, 2, 2) = -Abar;

        _Bin_vel(2*k) = _wheelVelMax*2.0*_wheelRadius;
        _Bin_vel(2*k+1) = _wheelVelMax*2.0*_wheelRadius;
        _Bin_vel(2*(k+_N)) = -_wheelVelMin*2.0*_wheelRadius;
        _Bin_vel(2*(k+_N)+1) = -_wheelVelMin*2.0*_wheelRadius;
    }

    // Check matrix
//    saveMatrixToFile("Ain_vel_matrix.csv", _Ain_vel);
//    saveMatrixToFile("Bin_vel_matrix.csv", _Bin_vel);
}

void MPC_diffDrive_fblin::saveMatrixToFile(std::string fileName, Eigen::MatrixXd matrix) {
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

void MPC_diffDrive_fblin::set_ErrorMsgCallback(ErrorMsgCallback callback) {
    _error = callback;
}

void MPC_diffDrive_fblin::set_InfoMsgCallback(InfoMsgCallback callback) {
    _info = callback;
}

void MPC_diffDrive_fblin::set_DebugMsgCallback(DebugMsgCallback callback) {
    _debug = callback;
}

void MPC_diffDrive_fblin::errorMsg(const std::string& message) {
    if (_error) {
        _error(message);
    }
}

void MPC_diffDrive_fblin::infoMsg(const std::string& message) {
    if (_info) {
        _info(message);
    }
}

void MPC_diffDrive_fblin::debugMsg(const std::string& message) {
    if (_debug) {
        _debug(message);
    }
}