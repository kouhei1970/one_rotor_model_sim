/******************************
 *                            
 * ルンゲクッタ法による 
 * マルチコプタシミュレーション       
 * サンプルプログラム            
 *                            
 ******************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h> //可変長引数関数を使うために必要
#include <math.h>
#include <time.h>

#define RADPS2RPM (60.0/2.0/3.14159)

enum 
{
  RIGHT,
  LEFT
};

//定数ノミナル
const double Lm = 3.7e-4;//Inductance[H]
const double Rm = 1.2e-1;//Resistance[Ohm]
const double Km = 3.3e-3;//Torque constant[Nm/A]
const double Jm = 8.1e-6;//Moment of inertia[kg m^2]
const double Cq = 3.0e-8;//Cofficient of torque (Propeller)
const double Dm = 0.0;   //Cofficient of viscous damping [Nm s]
const double Ct = 3.5e-6;//Cofficient thrust[N]
const double lcpt=0.09; //Drome arm length[m]
const double Jcpt=6.0e-3; //Drone moment of inertia
const double Md = 0.35;//Mass of drone
const double Grav = 9.80665; //Accelaration of gravity[m/s^2]
const double End_time = 2.0;//Time [s]
long cpu_time;

//モータ状態構造体
typedef struct 
{
  double i;
  double omega;
  double u;
  double i_;
  double omega_;
  double u_;
} motor_t;

//
//マルチコプタ座標系
// 
// 右が正、下が正
// 反時計回りが回転の正
//
// (o)------------>x
//  |
//  |
//  |
//  |
//  V
//  z

//マルチコプタ状態構造体
typedef struct
{
  double q;
  double theta;
  double u;
  double w;
  double x;
  double z; 
  double q_;
  double theta_;
  double u_;
  double w_;
  double x_;
  double z_; 
} drone_t;

//Equation of current
//Lm di/dt + Rm i + Km omega = u
//i:current
//t:time
//value[0]:omega
//value[1]:u
double i_dot(double i, double t, double *value)
{
  double omega = value[0];
  double u = value[1];
  return (u - Rm * i - Km * omega)/Lm;
}

//Motor Equation of motion
//TL = Cq omega^2
//Jm domega/dt + Dm omega + TL = Km i
//omega:angular velocity
//t:time
//value[0]:i
double omega_dot(double omega, double t, double *value)
{
  double i=value[0];
  double TL=Cq * omega * omega;
  return (Km * i - Dm * omega - TL)/Jm;
}

//Multcopter Equation of angular motion
//Jcpt dq/dt = (T_R-T_L) l
//omega_R:prop angular velocity
//omega_L:prop angular velocity
//t:time
//value[0]:omega_R
//value[1]:omega_L
double q_dot(double q, double t, double *value)
{
  double omega_R = value[0];
  double omega_L = value[1];
  double T_R = Ct * omega_R * omega_R;
  double T_L = Ct * omega_L * omega_L;
  return (T_R - T_L)*lcpt/Jcpt;
}

//Multcopter Equation of motion
//Md du/dt = -(T_R+T_L) + Md*grav*sin(theta)
//omega_R:angular velocity
//omega_L:angular velocity
//t:time
//value[0]:omega_R
//value[1]:omega_L
//value[2]:theta
double u_dot(double w, double t, double *value)
{
  double omega_R = value[0];
  double omega_L = value[1];
  double theta = value[2];
  double T_R = Ct * omega_R * omega_R;
  double T_L = Ct * omega_L * omega_L;
  return -Md*Grav*sin(theta)/Md;
}

//Multcopter Equation of motion
//Md du/dt = -(T_R+T_L) + Md*grav*sin(theta)
//omega_R:angular velocity
//omega_L:angular velocity
//t:time
//value[0]:omega_R
//value[1]:omega_L
//value[2]:theta
double w_dot(double w, double t, double *value)
{
  double omega_R = value[0];
  double omega_L = value[1];
  double theta = value[2];
  double T_R = Ct * omega_R * omega_R;
  double T_L = Ct * omega_L * omega_L;
  return (-(T_R + T_L) + Md*Grav*cos(theta))/Md;
}

//Multicopter kinematics
//dtheta/dt = q
//t:time
//value[0]:q
double theta_dot(double theta, double t, double *value)
{
  double q=value[0];
  return q;
}

//Multicopter kinematics
//dx/dt = u*cos(theta) + w*sin(theta)
//t:time
//value[0]:u
//value[1]:w
//value[2]:theta
double x_dot(double x, double t, double *value)
{
  double u = value[0];
  double w = value[1];
  double theta= value[2];
  return u*cos(theta) + w*sin(theta); 
}

//Multicopter kinematics
//dz/dt = -u*sin(theta) + w*cos(theta)
//t:time
//value[0]:u
//value[1]:w
//value[2]:theta
double z_dot(double z, double t, double *value)
{
  double u = value[0];
  double w = value[1];
  double theta= value[2];
  return -u*sin(theta) + w*cos(theta); 
}

//Runge Kutta method
//dxdy:derivative
//x:state
//t:time
//h:step size
//n:argument
double rk4(double (*dxdt)(double, double, double*), double x, double t, double h, int n, ...)
{
  va_list args;
  double *value;
  double k1,k2,k3,k4;

  value=(double*)malloc(sizeof(double) * n);
  va_start(args , n);
  for(int i=0;i<n;i++)
  {
    value[i]=va_arg(args, double);
  }
  va_end(args);
  
  k1 = h * dxdt(x, t, value);
  k2 = h * dxdt(x+0.5*h*k1, t+0.5*h, value);
  k3 = h * dxdt(x+0.5*h*k2, t+0.5*h, value);
  k4 = h * dxdt(x+h*k3, t+h, value);

  free(value);
  
  return x+(k1 + k2*2.0 + k3*2.0 + k4)/6;
}

void save_state(motor_t* motor, drone_t* drone)
{
  for (char i=RIGHT; i<LEFT+1; i++)
  {
    motor[i].i_ = motor[i].i;
    motor[i].omega_ = motor[i].omega;
    motor[i].u_ = motor[i].u;
  }
  drone->q_ = drone->q;
  drone->theta_ = drone->theta;
  drone->u_ = drone->u;
  drone->w_ = drone->w;
  drone->x_ = drone->x;
  drone->z_ = drone->z;
}

void print_state(double t, motor_t* motor, drone_t drone)
{
  printf("%11.8f %11.8f %11.8f %11.8f %11.8f %11.8f %11.8f %11.8f %11.8f %11.8f %11.8f\n",
    t, 
    motor[RIGHT].i, 
    motor[LEFT].i, 
    motor[RIGHT].omega,
    motor[LEFT].omega,
    drone.q,
    drone.theta,
    drone.u,
    drone.w,
    drone.x,
    drone.z
  );
}

void drone_sim(void)
{
  drone_t drone;
  motor_t motor[2];

  //state init
  motor[RIGHT].i = 0.0;
  motor[RIGHT].omega = 0.0;
  motor[RIGHT].u = 7.5;
  motor[LEFT].i  = 0.0;
  motor[LEFT].omega  = 0.0;
  motor[LEFT].u = 7.4;
  drone.q = 0.0;
  drone.theta = 0.0;

  double t       = 0.0; //time
  double h       = 0.0001;//step size

  //initial state output
  print_state(t, motor, drone);
  
  cpu_time = clock();
  while(t < End_time )    
  {
    //Save state
    save_state(motor, &drone);

    //Update(Runge-Kutta method)
    for (char i=RIGHT; i<LEFT+1; i++)
    {
      motor[i].i = rk4(i_dot, motor[i].i_, t, h, 2, motor[i].omega_, motor[i].u_);
      motor[i].omega = rk4(omega_dot, motor[i].omega_, t, h, 1, motor[i].i_);
    }
    drone.q = rk4(q_dot, drone.q_, t, h, 2, motor[RIGHT].omega_, motor[LEFT].omega_);
    drone.theta = rk4(theta_dot, drone.theta_, t, h, 1, drone.q_);
    drone.u = rk4(u_dot, drone.u_, t, h, 3, motor[RIGHT].omega_, motor[LEFT].omega_, drone.theta_);
    drone.w = rk4(w_dot, drone.w_, t, h, 3, motor[RIGHT].omega_, motor[LEFT].omega_, drone.theta_);
    drone.x = rk4(x_dot, drone.x_, t, h, 3, drone.u_, drone.w_, drone.theta_);
    drone.z = rk4(z_dot, drone.z_, t, h, 3, drone.u_, drone.w_, drone.theta_);

    t = t + h;
    
    //Output
    print_state(t, motor, drone);
  }
  cpu_time = clock() -cpu_time;
  //printf("elapsed time %f\n", (double)cpu_time/CLOCKS_PER_SEC);
}

void main(void)
{
  drone_sim();
}
