#ifndef FBLIN_UNICYCLE
#define FBLIN_UNICYCLE


class fblin_unicycle
{
public:
    fblin_unicycle(double P_distance);
    ~fblin_unicycle();

    void set_unicycleState(double position_x, double position_y, double heading);

    void control_transformation(double vPx, double vPy, double& linSpeed, double& angSpeed);
    void ouput_transformation(double& xP, double& yP);
    void reference_transformation(double x_ref, double y_ref, double theta_ref, double& xP_ref, double& yP_ref);
private:
    double p;
    double x, y, theta;
};

#endif /* FBLIN_UNICYCLE */