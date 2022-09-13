#include "propeller.hpp"

Propeller::Propeller(){};

void Propeller::set_parameter(double cq, double ct)
{
    Cq = cq;
    Ct = ct;
}

double Propeller::get_load_torque(double omega)
{
    return Cq*omega*omega;
}

double Propeller::get_thrust(double omega)
{
    return Ct*omega*omega;
}

