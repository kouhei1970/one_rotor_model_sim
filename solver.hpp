#ifndef SOLVER_HPP
#define SOLVER_HPP

class Solver
{
    private:

    public:
        Solver();
        double rk4(double (*dxdt)(double, double, double*), double x, double t, double h, int n, ...);
};

#endif