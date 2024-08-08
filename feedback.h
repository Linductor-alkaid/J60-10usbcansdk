#pragma once
#include <zlgcan/zlgcan.h>
struct motor_feedback
{
    int id;
    double position;
    double velocity;
    double torque;
};
