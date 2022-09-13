#ifndef AIRFRAME_HPP
#define AIRFRAME_HPP

class Airframe
{
    public:
        //定数ノミナル
        const double Md = 0.35;//Mass of drone
        const double Grav = 9.80665; //Accelaration of gravity[m/s^2]
        
        //状態
        double Um = 0.0;
        double Vm = 0.0;
        double Wm = 0.0;
        double Pm = 0.0;
        double Qm = 0.0;
        double Rm = 0.0;
        double Xm = 0.0;
        double Ym = 0.0;
        double Zm = 0.0;
        double Phim = 0.0;
        double Thetam = 0.0;
        double Psim = 0.0;
        double Q1m = 0.0;
        double Q2m = 0.0;
        double Q3m = 0.0;
        double Q4m = 0.0;

        //Private Method
        double w_dot(double w, double t, double *value);
        double z_dot(double z, double t, double *value);

        Airframe();

};

#endif