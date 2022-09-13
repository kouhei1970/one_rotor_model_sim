#include "drone.hpp"

Drone::Drone()
{
}
void Drone::update(void)
{
    double omega_, w_, z_;
    //Save state
    //save_state(&motor, &drone);

    // declare a pointer to member function
    double (Motor::*p_omega_dot) (double, double, double*) = &Motor::omega_dot;
    double (Airframe::*p_w_dot) (double, double, double*) = &Airframe::w_dot;
    double (Airframe::*p_z_dot) (double, double, double*) = &Airframe::z_dot;

    //double tmp[0];
    //(motor.*p_omega_dot)(0.0,0.0,tmp);

    //Update(Runge-Kutta method)
    omega_ = solver.rk4( (motor.*p_omega_dot), motor.omega, 0.0, 0.01, 1, motor.u);
    w_ = solver.rk4( (frame.*p_w_dot), frame.Wm, t, step, 1, motor.omega);
    z_ = solver.rk4( (frame.*p_z_dot), frame.Zm, t, step, 1, frame.Wm);
    motor.omega = omega_;
    frame.Wm = w_;
    frame.Zm = z_;

}
