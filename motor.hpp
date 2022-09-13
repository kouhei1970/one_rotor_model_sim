#ifndef MOTOR_HPP
#define MOTOR_HPP

class Motor
{
    private:
        //定数ノミナル
        const double Rm = 1.2e-1;//Resistance[Ohm]
        const double Km = 3.3e-3;//Torque constant[Nm/A]
        const double Jm = 8.1e-6;//Moment of inertia[kg m^2]
        const double Dm = 0.0;   //Cofficient of viscous damping [Nm s]

    public:
        //State variable
        double omega = 0.0;
        double u = 0.0;
        double omega_ = 0.0;
        double u_ = 0.0;

        Motor();
        double omega_dot(double omega, double t, double *value);


};

#endif