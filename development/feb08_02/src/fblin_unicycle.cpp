#include "fblin_unicycle.h"

#include <cmath>
#include <stdexcept>


fblin_unicycle::fblin_unicycle(double P_distance)
{
    // Check parameter consistency
    if (P_distance<=0.0)
        throw std::invalid_argument("[fblin_unicycle] The distance of point P should be strictly greater then zero");

    // Variables initialization
    p = P_distance;

    x = y = theta = 0.0;
}

fblin_unicycle::~fblin_unicycle()
{
    // Do nothing
}

void fblin_unicycle::set_unicycleState(double position_x, double position_y, double heading)
{
    // State update
    x = position_x;
    y = position_y;
    theta = heading;
}

void fblin_unicycle::control_transformation(double vPx, double vPy, double& linSpeed, double& angSpeed)
{
    linSpeed = vPx*cos(theta)+vPy*sin(theta);
    angSpeed = (vPy*cos(theta)-vPx*sin(theta))/p;
}

void fblin_unicycle::ouput_transformation(double& xP, double& yP)
{
    xP = x + p*cos(theta);
    yP = y + p*sin(theta);
}

void fblin_unicycle::reference_transformation(double x_ref, double y_ref, double theta_ref, double& xP_ref, double& yP_ref)
{
    xP_ref = x_ref + p*cos(theta_ref);
    yP_ref = y_ref + p*sin(theta_ref);
}