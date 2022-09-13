#include "motor.hpp"

Motor::Motor(){};


//Motor Equation of motion
//TL = Cq omega^2
//Jm domega/dt + (Dm + K^2/Rm) omega + TL = Km u/R
//omega:angular velocity
//u:value[0] Input voltage
//TL:value[1] Load torque
//t:time
double Motor::omega_dot(double omega, double t, double *value)
{
  double u =value[0];//Input voltage
  double TL = value[1];//Load torque
  //double TL=Cq * omega * omega;
  return (Km*u/Rm - (Dm + Km*Km/Rm)*omega - TL)/Jm;
}
