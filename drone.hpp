#ifndef DRONE_HPP
#define DRONE_HPP

#include "airframe.hpp"
#include "motor.hpp"
#include "propeller.hpp"
#include "solver.hpp"

class Drone
{
    public:
    Airframe frame;
    Motor motor;
    Propeller propeller;
    Solver solver;

    // 関数ポインタ

    public:
    Drone();
    void update();


};


#endif