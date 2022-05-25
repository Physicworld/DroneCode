#include "PID.h"

PID::PID(float _kp, float _ki, float _kd)
{
    this->kp = _kp;
    this->ki = _ki;
    this->kd = _kd;
}

float PID::p_controller(float error_actual)
{
    return this->kp * error_actual;
}

float PID::i_controller(float error_actual)
{
    this->integral += error_actual;
    return this->ki * this->integral;
}

float PID::d_controller(float error_actual, float tiempo_actual)
{
    return this->kd * (error_actual - this->error_previo) / (tiempo_actual - this->tiempo_previo);
}

float PID::compute_pid(float error_actual, float tiempo_actual)
{
    float pid = p_controller(error_actual) + i_controller(error_actual) + d_controller(error_actual, tiempo_actual);
    this->error_previo = error_actual;
    this->tiempo_previo = tiempo_actual;
    return pid;
}