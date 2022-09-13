#ifndef PROPELLER_HPP
#define PROPELLER_HPP

class Propeller
{
    private:
        const double Cq = 3.0e-8;//Cofficient of torque (Propeller)
        const double Ct = 3.5e-6;//Cofficient thrust[N]3.5e-6
    public:

    Propeller();
    void set_parameter(double cq, double ct);
    double get_load_torque(double omega);
    double get_thrust(double omega);

};

#endif