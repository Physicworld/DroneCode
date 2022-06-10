#include <Wire.h>
#include <Servo.h>

Servo moto1;
Servo moto2;
Servo moto3;
Servo moto4;

int i;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acc_angulo[2];
float Gyro_angulo[2];
float Angulo_total[2];

float time, tiempoPrev, tiempo_transcurrido;

float radian_grado = 180 / 3.141592654;
float PIDy, PIDx, errory, errorx, errorPrevio;
int pwmoto1, pwmoto2, pwmoto3, pwmoto4;
float pid_py = 0;
float pid_iy = 0;
float pid_dy = 0;

float pid_px = 0;
float pid_ix = 0;
float pid_dx = 0;

double kp = 1.8;
double ki = 0; //0.003
double kd = 0;

int estado = 0, opcion = 0;
double acelerador = 1150;
float anguloRef = 0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  moto1.attach(3);
  moto2.attach(5);
  moto3.attach(6);
  moto4.attach(9);

  time = millis();

  moto1.writeMicroseconds(1000);
  moto2.writeMicroseconds(1000);
  moto3.writeMicroseconds(1000);
  moto4.writeMicroseconds(1000);
  delay(3000);

}

void loop() {

  while (Serial.available () > 0 ) {
    estado = Serial.read();
    estado -= 48;
    Serial.println(estado);
  }
  if (estado == 3) {
    opcion = 3;
  }

  if (estado == 4) {
    opcion = 4;
  }

  if (opcion == 3) {

    if ( estado == 1) {
      acelerador += 10;
      estado = 0;
      //  if( acelerador > 2000){
      //  acelerador = 2000;
      //}
      // Serial.println(acelerador);
    }


    if ( estado == 2) {
      acelerador -= 10;
      estado = 0;
      //if(acelerador < 1000){
      //acelerador = 1000;
      //}
      // Serial.println(acelerador);
    }

    PID(acelerador, opcion);

  }
  if (opcion == 4) {
    PID(acelerador, opcion);
  }
}

void PID(double acelerador, int opcion) {
  delay(100);
  tiempoPrev = time;
  time = millis();
  tiempo_transcurrido = ( time - tiempoPrev) / 1000;

  Wire.beginTransmission(0x69);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x69, 6, true);

  Acc_rawX = Wire.read() << 8 | Wire.read();
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();
  Serial.print(Acc_rawX);
  //X
  Acc_angulo[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * radian_grado;
  //Y
  Acc_angulo[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * radian_grado;

  Wire.beginTransmission(0x69);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x69, 4, true);

  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();
  //X
  Gyro_angulo[0] = Gyr_rawX / 131.0;
  //Y
  Gyro_angulo[1] = Gyr_rawY / 131.0;

  // X angulo
  Angulo_total[0] = 0.98 * (Angulo_total[0] + Gyro_angulo[0] * tiempo_transcurrido) + 0.02 * Acc_angulo[0];
  //Y angulo
  Angulo_total[1] = 0.98 * (Angulo_total[1] + Gyro_angulo[1] * tiempo_transcurrido) + 0.02 * Acc_angulo[1];
  Serial.print("  X:");
  Serial.print(Angulo_total[0]);
  Serial.print("  Y:");
  Serial.println(Angulo_total[1]);
  errorx = Angulo_total[0] - anguloRef;
  errory = Angulo_total[1] - anguloRef;

  pid_px = kp * errorx;
  pid_py = kp * errory;

  if (-3 < errory < 3) {
    pid_iy = pid_iy + (ki * errory);
  }

  if (-3 < errorx < 3) {
    pid_ix = pid_ix + (ki * errorx);
  }

  pid_dy = kd * (( errory - errorPrevio) / tiempo_transcurrido);
  pid_dx = kd * ((errorx - errorPrevio) / tiempo_transcurrido);

  PIDy = pid_py + pid_iy + pid_dy;
  PIDx = pid_px + pid_ix + pid_dx;

  if (PIDy < -1000) {
    PIDy = -1000;
  }

  if (PIDx < -1000) {
    PIDx = -1000;
  }

  if (PIDy > 1000) {
    PIDy = 1000;
  }

  if (PIDx > 1000) {
    PIDx = 1000;
  }

  pwmoto1 = acelerador + PIDy;
  pwmoto3 = acelerador - PIDy;
  pwmoto2 = acelerador + PIDx;
  pwmoto4 = acelerador - PIDx;

  // Estableciendo valores maximos
  if (pwmoto1 > 2000) {
    pwmoto1 = 2000;
  }
  if (pwmoto2 > 2000) {
    pwmoto2 = 2000;
  }
  if (pwmoto3 > 2000) {
    pwmoto3 = 2000;
  }
  if (pwmoto4 > 2000) {
    pwmoto4 = 2000;
  }

  // Estableciendo valores minimos
  if (pwmoto1 < 1000) {
    pwmoto1 = 1000;
  }
  if (pwmoto2 < 1000) {
    pwmoto2 = 1000;
  }
  if (pwmoto3 < 1000) {
    pwmoto3 = 1000;
  }
  if (pwmoto4 < 1000) {
    pwmoto4 = 1000;
  }

  Serial.println(pwmoto1);
  Serial.println(pwmoto2);
  Serial.println(pwmoto3);
  Serial.println(pwmoto4);
  MOTO(pwmoto1, pwmoto2, pwmoto3, pwmoto4);

  if (opcion == 4) {
    MOTO(1000, 1000, 1000, 1000);
  }
}

void MOTO(int pwmoto1, int pwmoto2, int pwmoto3, int pwmoto4) {
  moto1.writeMicroseconds(pwmoto1);
  moto2.writeMicroseconds(pwmoto2);
  moto3.writeMicroseconds(pwmoto3);
  moto4.writeMicroseconds(pwmoto4);

}
