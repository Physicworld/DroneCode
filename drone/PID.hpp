class PID
{
  public:
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float integral = 0;
    float error_previo = 0;
    float tiempo_previo = 0;

    PID(float _kp, float _ki, float _kd)
    {
      this->kp = _kp;
      this->ki = _ki;
      this->kd = _kd;
    }
    float p_controller(float error_actual)
    {
      return this->kp * error_actual;
    }
    float i_controller(float error_actual)
    {
      this->integral += (this->ki * this->integral);
      return this->integral;
    }

    float d_controller(float error_actual, float tiempo_actual)
    {
      return this->kd * (error_actual - this->error_previo) / (tiempo_actual - this->tiempo_previo);
    }

    float compute_pid(float error_actual, float tiempo_actual)
    {
      float pid = p_controller(error_actual) + i_controller(error_actual) + d_controller(error_actual, tiempo_actual);
      this->error_previo = error_actual;
      this->tiempo_previo = tiempo_actual;
      return pid;
    }
};
