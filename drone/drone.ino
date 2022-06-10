#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include "PID.hpp"
#include "Vector.hpp"

//Declaramos variable para cada motor
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

//Declaramos el Bluetooth
SoftwareSerial BT(9, 8); // RX | TX

// Declaro vector de angulo actual
Vector current_angle;

//Variables de conversion
float radian_grado = 180 / 3.141592654;

//Declaramos variables de tiempo
float time, tiempo_previo, tiempo_transcurrido;

//Direccion del modulo mpu6050
const int mpu_address = 0x68;

//Variables de control
Vector k_pid(5, 1, 1.5);
Vector angulo_ref(0, 0, 0);
Vector error_axis(0, 0, 0);
Vector pid_signal(0, 0, 0);
int current_state = 0;

//Declaramos pid controller para eje X y eje Y
// X
PID PIDx(k_pid.x, k_pid.y,  k_pid.z);
//Y
PID PIDy(k_pid.x, k_pid.y, k_pid.z);

//Motores
float acelerador = 1000;
int pwmotor1, pwmotor2, pwmotor3, pwmotor4;

void setup()
{
  //Inicializamos comunicaciones I2C
  Wire.begin();
  Wire.beginTransmission(mpu_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //Inicializamos comunicaciones Seriales
  Serial.begin(9600);
  BT.begin(9600);

  //Inicializamos motores
  motor1.attach(3);
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(10);

  //Damos señal de inicio a motores
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(3000);
}

void loop()
{
  time = millis();
  tiempo_previo = time / 1000;
  tiempo_transcurrido = (time - tiempo_previo * 1000) / 1000;
  current_state = read_serial();
  current_angle = read_gyroscope(current_angle, tiempo_transcurrido);
  current_angle.print_vector();
  error_axis.x = current_angle.x - angulo_ref.x;
  error_axis.y = current_angle.y - angulo_ref.y;
  pid_signal.x = PIDx.compute_pid(error_axis.x, tiempo_previo);
  pid_signal.y = PIDy.compute_pid(error_axis.y, tiempo_previo);
  control_motors(current_state, pid_signal);

  //delay(10);
}

void control_motors(int current_state, Vector pid_signal)
{
  if (pid_signal.x < -1000)
  {
    pid_signal.x = -1000;
  }
  if (pid_signal.y < -1000)
  {
    pid_signal.y = -1000;
  }

  if (pid_signal.x > 1000)
  {
    pid_signal.x = 1000;
  }
  if (pid_signal.y > 1000)
  {
    pid_signal.y = 1000;
  }

  pwmotor1 = acelerador - pid_signal.y; //3
  pwmotor2 = acelerador + pid_signal.y; //5
  pwmotor3 = acelerador + pid_signal.x; //6
  pwmotor4 = acelerador - pid_signal.x; //10

  // Estableciendo valores maximos
  if (pwmotor1 > 2000) {
    pwmotor1 = 2000;
  }
  if (pwmotor2 > 2000) {
    pwmotor2 = 2000;
  }
  if (pwmotor3 > 2000) {
    pwmotor3 = 2000;
  }
  if (pwmotor4 > 2000) {
    pwmotor4 = 2000;
  }

  // Estableciendo valores minimos
  if (pwmotor1 < 1000) {
    pwmotor1 = 1000;
  }
  if (pwmotor2 < 1000) {
    pwmotor2 = 1000;
  }
  if (pwmotor3 < 1000) {
    pwmotor3 = 1000;
  }
  if (pwmotor4 < 1000) {
    pwmotor4 = 1000;
  }

  switch (current_state)
  {
    case 1:
      break;
    case 2:
      acelerador += 5;
      break;
    case 3:
      break;
    case 4:
      acelerador -= 5;
      break;
    case 5:
      acelerador = 1000;
      break;
    case 6:
      acelerador = 1150;
      break;
  }
  modify_motors();
}

void modify_motors()
{
  motor1.writeMicroseconds(pwmotor1);
  motor2.writeMicroseconds(pwmotor2);
  motor3.writeMicroseconds(pwmotor3);
  motor4.writeMicroseconds(pwmotor4);
}

int read_serial()
{
  int state = 0;
  while (BT.available() > 0)
  {
    state = BT.read() - 48;
  }
  return state;
}

Vector read_gyroscope(Vector current_angle, float tiempo_transcurrido)
{
  //Vectores sobre los cuales haremos operaciones.
  Vector acc_angle;
  Vector gyro_angle;
  //Vector current_angle;

  // Variables para leer el modulo.
  int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;

  // Iniciamos comunicacion I2C con el modulo mpu6050
  Wire.beginTransmission(mpu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 6, true);

  //Leemos los primeros 24 bits, contienen informacion de la aceleracion.
  Acc_rawX = Wire.read() << 8 | Wire.read();
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  //Hacemos el calculo del vector de aceleracion
  //X
  acc_angle.x = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * radian_grado;
  //Y
  acc_angle.y = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * radian_grado;

  //Estanblecemos de nuevo comunicacion con el modulo mpu.
  Wire.beginTransmission(mpu_address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 4, true);
  //Leemos los siguientes 16 bits, contienen informacion de los angulos.
  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();
  //X
  gyro_angle.x = Gyr_rawX / 131.0;
  //Y
  gyro_angle.y = Gyr_rawY / 131.0;

  //Calculamos el angulo actual
  // X angulo
  current_angle.x = 0.98 * (current_angle.x + gyro_angle.x * tiempo_transcurrido) + 0.02 * acc_angle.x;
  //Y angulo
  current_angle.y = 0.98 * (current_angle.y + gyro_angle.y * tiempo_transcurrido) + 0.02 * acc_angle.y;
  //retornamos estructura con informacion del angulo actual
  return current_angle;
}
