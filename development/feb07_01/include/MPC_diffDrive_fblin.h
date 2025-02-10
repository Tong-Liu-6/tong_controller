#ifndef MPC_DIFFDRIVE_FBLIN_H
#define MPC_DIFFDRIVE_FBLIN_H

#include <boost/function.hpp>
#include <Eigen/Dense>

#include "GUROBIsolver.h"
#include "fblin_unicycle.h"

// #define GUROBI_LICENSEID 960125
// #define GUROBI_USERNAME  "bascetta"

#define GUROBI_LICENSEID 2610262
#define GUROBI_USERNAME  "tong6"

// Message handlers
typedef boost::function<void(const std::string &)> DebugMsgCallback;
typedef boost::function<void(const std::string &)> InfoMsgCallback;
typedef boost::function<void(const std::string &)> ErrorMsgCallback;


/**
 * @class MPC_diffDrive_fblin
 * @brief MPC regulator/trajectory tracking controller with feedback linearization for differential drive robots
 */
class MPC_diffDrive_fblin {

public:
    /**
     * @brief Constructor for MPC_diffDrive_fblin
     */
    MPC_diffDrive_fblin();

    /**
     * @brief Destructor for MPC_diffDrive_fblin
     */
    ~MPC_diffDrive_fblin();

    /**
     * @brief Set robot model parameters
     * @param wheelVelMax maximum wheel angular velocity
     * @param wheelVelMin minimum wheel angular velocity
     * @param wheelRadius wheel nominal radius
     * @param track distance between wheel contact points
     */
    void set_robotParams(double wheelVelMax, double wheelVelMin, double wheelRadius, double track, double wheelAccMax);

    /**
     * @brief Set MPC controller parameters
     * @param samplingTime controller sampling time (must be an integer multiple of feedback linearization sampling time)
     * @param predictionHorizon length of the prediction horizon (number of samples)
     * @param q diagonal element of Q matrix
     * @param r diagonal element of R matrix
     * @param variableLB lower bounds of optimization variables
     * @param variableUB upper bounds of optimization variables
     */
    void set_MPCparams(double samplingTime, int predictionHorizon, double q, double r);
    void set_MPCparams(double samplingTime, int predictionHorizon, double q, double r,
                       const std::vector<double> &variableLB, const std::vector<double> &variableUB);

    /**
     * @brief Set feedback linearization parameters
     * @param samplingTime feedback linearization sampling time
     * @param pointPdistance distance of point P with respect to the wheel axis
     */
    void set_FBLINparams(double samplingTime, double pointPdistance);

    /**
     * @brief Initialize MPC controller
     * @return          false in case of an error, true otherwise
     */
    bool initialize();

    /**
     * @brief Executes MPC controller computing control values (x/y velocity of point P)
     * @return          false in case of an error, true otherwise
     */
    bool executeMPCcontroller();

    /**
     * @brief Executes the feedback linearizing controller computing robot velocities from x/y velocity of point P
     * @return          false in case of an error, true otherwise
     */
    bool executeLinearizationController();

    /**
     * @brief Set the actual robot pose
     * @param x actual robot x-position
     * @param y actual robot y-position
     * @param yaw actual robot orientation
     */
    void set_actualRobotState(double x, double y, double yaw);

    /**
     * @brief Set the reference robot state for a regulation problem (constant reference along the prediction horizon)
     * @param x reference robot x-position
     * @param y reference robot y-position
     * @param yaw reference robot orientation
     */
    void set_referenceRobotState(double x, double y, double yaw);

    /**
     * @brief Set the reference robot state along the prediction horizon for a trajectory tracking problem
     * @param refRobotState vector of reference robot states [x(k), y(k), yaw(k), ..., x(k+N), y(k+N), yaw(k+N)]
     * where N is the length of the prediction horizon
     */
    void set_referenceRobotState(const Eigen::VectorXd &refRobotState);

    /**
     * @brief Retrieve the actual MPC state (point P position)
     * @param xP point P x-position
     * @param yP point P y-position
     */
    void get_actualMPCstate(double& xP, double& yP);

    /**
     * @brief Retrieve the reference MPC state (point P position) along the prediction horizon
     * @param refState vector of reference states (N+1 elements, where N is the length of the prediction horizon)
     */
    void get_referenceMPCstate(Eigen::VectorXd &refState);

    /**
     * @brief Retrieve the actual MPC control vector (velocity of point P)
     * @param vPx x-component of point P velocity
     * @param vPy y-component of point P velocity
     */
    void get_actualMPCControl(double& vPx, double& vPy);

    /**
     * @brief Retrieve the actual robot control vector (linear and angular robot velocity)
     * @param linVelocity linear velocity of the robot
     * @param angVelocity angular velocity of the robot
     */
    void get_actualControl(double &linVelocity, double &angVelocity);

    /**
     * @brief Callbacks used by the class to print error/info/debug messages
     * The callback functions should have the following prototype
     * void debugMsgCallback(const std::string &message)
     * void infoMsgCallback(const std::string &message)
     * void errorMsgCallback(const std::string &message)
     * @param callback a pointer to the callback function that prints the message
     */
    void set_ErrorMsgCallback(ErrorMsgCallback callback);
    void set_InfoMsgCallback(InfoMsgCallback callback);
    void set_DebugMsgCallback(DebugMsgCallback callback);

    void reset_pre_vP();
    void set_obstacleConstraint(int obstacle_flag, double Ax, double Ay, double B);

private:
    // MPC parameters
    int _N;
    double _MPC_Ts;
    double _q, _r, _k, _p;
    bool _MPCparamsInitialized;
    double A_obs_x, A_obs_y, B_obs;

    Eigen::MatrixXd _plant_A, _plant_B;
    std::vector<double> _lowerBound, _upperBound;

    Eigen::MatrixXd _Acal, _Bcal;
    Eigen::MatrixXd _Qcal, _Rcal;
    Eigen::MatrixXd _H;
    Eigen::VectorXd _f;
    Eigen::MatrixXd _Ain_tot;
    Eigen::VectorXd _Bin_tot;

    double _actXP, _actYP, _actX, _actY, _actYaw;
    Eigen::VectorXd _predictRobotState, _refMPCstate, _optimVect;

    GUROBIsolver *_solver;

    // Feedback linearization parameters
    double _fblin_Ts, _Pdist;

    fblin_unicycle *_fblinController, *_fblinSimController;
    bool _FBLINparamsInitialized;

    // Robot parameters
    double _wheelVelMax, _wheelVelMin;
    double _track, _wheelRadius;
    double _wheelAccMax;
    bool _robotParamsInitialized;

    // Controller variables
    bool _controllerInitialized;
    double _linearVelocity, _angularVelocity;
    double pre_vPx, pre_vPy;

    // Controller constraints
    std::vector<GRBConstr> constraintMatrix;

    // Message handler function pointers
    DebugMsgCallback _debug;
    InfoMsgCallback _info;
    ErrorMsgCallback _error;

    // Private member functions
    void compute_AcalMatrix();
    void compute_BcalMatrix();
    void compute_QcalMatrix();
    void compute_RcalMatrix();
    void compute_objectiveMatrix();
    void compute_constraintMatrix();

    void saveMatrixToFile(std::string fileName, Eigen::MatrixXd matrix);

    void errorMsg(const std::string &message);
    void infoMsg(const std::string &message);
    void debugMsg(const std::string &message);

};

#endif //MPC_DIFFDRIVE_FBLIN_H
