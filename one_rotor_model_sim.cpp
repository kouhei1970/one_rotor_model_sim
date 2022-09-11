/******************************
 *                            
 * ルンゲクッタ法による 
 * マルチコプタの高度制御シミュレーション       
 * サンプルプログラム            
 *                            
 ******************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h> //可変長引数関数を使うために必要
#include <math.h>
#include <time.h>
#include "pid.hpp"

#define RADPS2RPM (60.0/2.0/3.14159)

//以下の行のコメントを外すことで速度制御が加わる
//#define WITH_VELOCITY_CONTROL

//定数ノミナル
const double Rm = 1.2e-1;//Resistance[Ohm]
const double Km = 3.3e-3;//Torque constant[Nm/A]
const double Jm = 8.1e-6;//Moment of inertia[kg m^2]
const double Cq = 3.0e-8;//Cofficient of torque (Propeller)
const double Dm = 0.0;   //Cofficient of viscous damping [Nm s]
const double Ct = 3.5e-6;//Cofficient thrust[N]3.5e-6
const double Md = 0.35;//Mass of drone
const double Grav = 9.80665; //Accelaration of gravity[m/s^2]
const double End_time = 20.0;//Time [s]
long cpu_time;

//モータ状態構造体
typedef struct 
{
  double omega;
  double u;
  double omega_;
  double u_;
} motor_t;

//マルチコプタ状態構造体
typedef struct
{
  double w;
  double z;
  double w_ref;
  double z_ref;
  double w_;
  double z_; 
} drone_t;

//Motor Equation of motion
//TL = Cq omega^2
//Jm domega/dt + (Dm + K^2/Rm) omega + TL = Km u/R
//omega:angular velocity
//u:value[0]=u
//t:time
double omega_dot(double omega, double t, double *value)
{
  double u =value[0];
  double TL=Cq * omega * omega;
  return (Km*u/Rm - (Dm + Km*Km/Rm)*omega - TL)/Jm;
}


//Multcopter Equation of motion
//Md dw/dt = 4*T - M g
//omega:angular velocity
//t:time
//value[0]:omega
double w_dot(double w, double t, double *value)
{
  double omega = value[0];
  double T = Ct * omega * omega;
  return (4*T - Md*Grav)/Md;
}

//Multicopter kinematics
//dz/dt = w
//t:time
//value[0]:w
double z_dot(double z, double t, double *value)
{
  double w = value[0];
  return w; 
}

//Runge Kutta method
//dxdy:derivative
//x:state
//t:time
//h:step size
//n:number of arguments
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
  motor->omega_ = motor->omega;
  motor->u_ = motor->u;
  drone->w_ = drone->w;
  drone->z_ = drone->z;
}

void print_state(double t, motor_t* motor, drone_t drone)
{
  #ifdef WITH_VELOCITY_CONTROL
    printf("%11.8f %11.8f %11.8f %11.8f %11.8f %11.8f\n",
      t, 
      motor->u,
      motor->omega,
      drone.w,
      drone.z,
      drone.w_ref
    );
  #else
    printf("%11.8f %11.8f %11.8f %11.8f %11.8f\n",
    t, 
    motor->u,
    motor->omega,
    drone.w,
    drone.z
  );
  #endif
}

//
//Drone simulator main
//
void drone_sim(void)
{
  drone_t drone;
  motor_t motor;
  PID alt_pid;
  PID w_pid;
  

  //state init
  motor.omega = 0.5*sqrt(Md*Grav/Ct);
  double u_trim = Rm*((Dm + Km*Km/Rm)*motor.omega + Cq*motor.omega*motor.omega)/Km;
  //u_trim = u_trim + 0.01*u_trim;
  motor.u = u_trim;
  drone.w = 0.0;
  drone.z = 0.0;
  drone.w_ref = 0.0;
  
  double t       = 0.0; //time
  double step    = 0.0001;//step size
  double h = 0.02; //control period
  double zref = 500.0;//(mm)
  double alt_err, w_err, w_ref;
  double next_control_time =0.0;

  uint8_t flag = 1;

  //PID parameter setup
  //PIDゲインの調整はここで行う
  //前から比例ゲイン，積分時間，微分時間，フィルタ時定数，制御周期
  #ifdef WITH_VELOCITY_CONTROL
    alt_pid.set_parameter(0.002, 100.0, 0.01, 0.01, h);
    w_pid.set_parameter(1.0, 1.0, 0.0000, 0.01, h);
    alt_pid.reset();
    w_pid.reset();
  #else
    alt_pid.set_parameter(0.0001, 1.0, 2.0, 0.01, h);
    alt_pid.reset();
  #endif
  //PID Gain print
  alt_pid.printGain();
  w_pid.printGain();

  //initial state output
  print_state(t, &motor, drone);
  
  //Simulation loop
  cpu_time = clock();
  while(t < End_time )    
  {
    //Control
    if (drone.z*1000.0>zref)flag =1;
    if(t>next_control_time)
    { 
      #ifdef WITH_VELOCITY_CONTROL
        if(flag ==1){
          alt_err = zref - (int)(drone.z*1000.0);
          drone.w_ref = alt_pid.update(alt_err);
          w_err = drone.w_ref - drone.w;
          motor.u = w_pid.update(w_err) + u_trim*0.95;
          if (motor.u > 7.4) motor.u = 7.4;
          else if (motor.u < 0.0) motor.u = 0.0;
        }
      #else
        if(flag ==1){
          alt_err = zref - (int)(drone.z*1000.0);
          motor.u = alt_pid.update(alt_err) + u_trim*0.95;
          if (motor.u > 7.4) motor.u = 7.4;
          else if (motor.u < 0.0) motor.u = 0.0;
        }
      #endif
      next_control_time = next_control_time + h;

    }

    //Save state
    save_state(&motor, &drone);

    //Update(Runge-Kutta method)
    motor.omega = rk4(omega_dot, motor.omega_, t, step, 1, motor.u_);
    drone.w = rk4(w_dot, drone.w_, t, step, 1, motor.omega_);
    drone.z = rk4(z_dot, drone.z_, t, step, 1, drone.w_);

    t = t + step;
    
    //Output
    print_state(t, &motor, drone);
  }
  cpu_time = clock() -cpu_time;
  printf("#elapsed time %f\n", (double)cpu_time/CLOCKS_PER_SEC);
}

int main(void)
{
  drone_sim();
}
