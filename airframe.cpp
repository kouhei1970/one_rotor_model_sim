#include "airframe.hpp"

Airframe::Airframe(){};

//Airframe Equation of motion
//Md dw/dt = 4*T - M g
//omega:angular velocity
//t:time
//value[0]:omega
double Airframe::Airframe(){};
double Airframe::w_dot(double w, double t, double *value)
{
  double omega = value[0];
  double T = Ct * omega * omega;
  return (4*T - Md*Grav)/Md;
}

//Airframe kinematics
//dz/dt = w
//t:time
//value[0]:w
double Airframe::z_dot(double z, double t, double *value)
{
  double w = value[0];
  return w; 
}

