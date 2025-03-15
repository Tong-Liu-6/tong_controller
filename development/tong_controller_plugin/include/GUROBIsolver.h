#ifndef GUROBISOLVER_H
#define GUROBISOLVER_H

#include "gurobi_c++.h"
#include <Eigen/Dense>
#include <boost/function.hpp>


// Message handlers
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;


class GUROBIsolver {

public:
    GUROBIsolver(int licenseID, std::string username);
    ~GUROBIsolver();

    bool initProblem(unsigned int numVariable, const std::vector<double>& lowerBound, const std::vector<double>& upperBound);

    bool setObjective(const Eigen::MatrixXd& hessian, const Eigen::VectorXd& gradient);

    bool addConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::vector<GRBConstr>& constraint);
    bool addConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::string name, std::vector<GRBConstr>& constraint);
    bool modifyConstraint(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, std::vector<GRBConstr>& constraint);
    bool removeConstraint(std::vector<GRBConstr>& constraint);

    bool solveProblem(Eigen::VectorXd& result, double& objectiveValue, int& optimizerStatus);

    bool writeProblem(const std::string& filename);
    bool printConstraint(const std::vector<GRBConstr>& constraint);

    enum optimizationStatus {OPTIMAL, INFEASIBLE, OTHER};

    void setErrorMsgCallback(ErrorMsgCallback callback);
    void setInfoMsgCallback(InfoMsgCallback callback);
    void setDebugMsgCallback(DebugMsgCallback callback);

private:
    bool _GUROBIinitialized;

    GRBEnv* _pEnv;
    GRBModel* _pModel;
    int _licenseID;
    std::string _username;

    std::vector<GRBVar> _variableVect;

    // Message handler function pointers
    DebugMsgCallback _debug;
    InfoMsgCallback _info;
    ErrorMsgCallback _error;

    void errorMsg(const std::string& message);
    void infoMsg(const std::string& message);
    void debugMsg(const std::string& message);
};

#endif //GUROBISOLVER_H
