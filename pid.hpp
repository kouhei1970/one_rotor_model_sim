#ifndef PID_HPP
#define PID_HPP

class PID
{
  private:
    double m_kp;
    double m_ti;
    double m_td;
    double m_filter_time_constant;
    double m_err,m_err2,m_err3;
    double m_h;
  public:
    double m_filter_output;
    double m_integral;
    PID();
    void set_parameter(
        double kp, 
        double ti, 
        double td,
        double filter_time_constant, 
        double h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    double filter(double x);
    double update(double err);
};

class Filter
{
  private:
    double m_state;
    double m_T;
    double m_h;
  public:
    double m_out;
    Filter();
    void set_parameter(
        double T,
        double h);
    void reset(void);
    double update(double u);
};

#endif