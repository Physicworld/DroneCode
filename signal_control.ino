

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
      this->integral += error_actual;
      return this->ki * this->integral;
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

PID pid_controller(0.1,0.05,0.005);
float error = 0;
float valor_deseado = 0;
float valor_medido;
float tiempo_actual = 0;
float u = 0;
float retroalimentacion = 0;
const int signalRef = A0;
const int outputControl = A1;
const int inputControl = 3;

void setup() {
  tiempo_actual = millis();
}

void loop() {

  tiempo_actual = millis();
  valor_deseado = analogRead(signalRef);
  valor_medido = analogRead(outputControl);
  error = valor_deseado - valor_medido;
  retroalimentacion = pid_controller.compute_pid(error, tiempo_actual);
  //CHECAR SATURACIONES
  u += retroalimentacion;
  if ( u > 255)
  {
    u = 255;
  }
  else if (u < 0)
  {
    u = 0;
  }
  analogWrite(inputControl, u);
}