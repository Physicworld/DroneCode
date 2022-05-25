
class PID
{
public:
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float integral = 0;
    float error_previo = 0;
    float tiempo_previo = 0;

    PID(float _kp, float _ki, float _kd);
    float p_controller(float error_actual);
    float i_controller(float error_actual);
    float d_controller(float error_actual, float tiempo_actual);
    float compute_pid(float error_actual, float tiempo_actual);
};