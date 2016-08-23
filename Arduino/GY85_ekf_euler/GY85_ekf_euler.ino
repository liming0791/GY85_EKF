/* Benchmark: Using GY-85 and TinyEKF on Arduino/Teensy.

   This code is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as
   published by the Free Software Foundation, either version 3 of the
   License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this code.  If not, see <http:#www.gnu.org/licenses/>.
*/

/*
  Arduino     MARG GY-85
  A5            SCL
  A4            SDA
  3.3V          VCC
  GND           GND
*/

#define N 3     // states, will also be measurement values M
#define M 3
#include <Wire.h>
#include <TinyEKF.h>
#include <ADXL345.h>  // ADXL345 Accelerometer Library
#include <HMC5883L.h> // HMC5883L Magnetometer Library
#include <ITG3200.h>
//#define DEBUG   // comment this out to do timing

//time count
unsigned long time, looptime;

//sensors
ADXL345 acc; //variable adxl is an instance of the ADXL345 library
HMC5883L compass;
ITG3200 gyro = ITG3200();
int error = 0;

//gyro
float goffsetX = 1.2f, goffsetY = 1.6f, goffsetZ = -0.58f;  //you should calibrate your own sensor
float  gx, gy, gz;
float  gx_rate, gy_rate, gz_rate;
float  gx_rateL = 0, gy_rateL = 0, gz_rateL = 0;
float  gx_rateR = 0, gy_rateR = 0, gz_rateR = 0;

//acc
float aoffsetX = 1.5f, aoffsetY = 16.5f, aoffsetZ = 245.f;  //you should calibrate your own sensor
int ax, ay, az;
int rawX, rawY, rawZ;
float X, Y, Z;
float rollrad, pitchrad;
float rolldeg, pitchdeg;

//mag
// Set declination angle on your location and fix heading
// You can find your declination on: http://magnetic-declination.com/
// (+) Positive or (-) for negative
// For HangZhou / ZheJiang / China declination angle is -5'22W (negtive)
// Formula: (deg + (min / 60.0)) / (180 / M_PI);
float declinationAngle = -(5.0 + (22.0 / 60.0)) / (180 / M_PI);
Vector norm;
float yawrad , yawdeg;
float yawrad2 , yawdeg2;

//==========================================================================

class Fuser : public TinyEKF {
  public:
    Fuser()
    {
      // We approximate the process noise using a small constant
      for (int j = 0; j < N; ++j)
          this->setQ(j, j, .001);
      // Same for measurement noise
      for (int j = 0; j < M; ++j)
        this->setR(j, j, .01);
      //Set P
      for (int j = 0; j < N; ++j)
        this->setP(j, j, 360);
    }
  protected:
    void model(double fx[N], double F[N][N], double hx[M], double H[M][N])
    {
      //states
      for (int j = 0; j < N; ++j) {
        // Process model is f(x) = x
        fx[j] = x[j] + looptime * gx_rate / 1000;
        // So process model Jacobian is identity matrix
        F[j][j] = 1;
      }

      //measurement
      for (int j = 0; j < M; ++j) {
        // Measurement function
        hx[j] = this->x[j];
        // Jacobian of measurement function
        H[j][j] = 1;
      }
    }
};

//EKF
Fuser ekf;
double z[M];

//=============================================================================//

void setupSensor() {
  //acc
  acc.powerOn();

  //COMPASS
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(64, -528, -124);

  //GYRO
  delay(1000);
  gyro.init(ITG3200_ADDR_AD0_LOW);
}

void xproduct(float* a, float* b, float* c){
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

void projectN(float* a, float* n, float* b){
  float l = sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
  float nx = n[0]/l;
  float ny = n[1]/l;
  float nz = n[2]/l;

  float direct = a[0] * nx + a[1] * ny + a[2]* nz;
  b[0] = a[0] - direct * nx;
  b[1] = a[1] - direct * ny;
  b[2] = a[2] - direct * nz;
}

float computeYaw(float* a, float* b, float* n){
  float la = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  float lb = sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);

  float cosYaw = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2])/(la*lb);
  float angle = acos(cosYaw)*180/PI;

  float xpro[3];
  xproduct(a, b, xpro);
  if(xpro[0]*n[0]+xpro[1]*n[1]+xpro[2]*n[2]>0){
    return angle;
  }else{
    return -angle;
  }
}

void getPose() {
  //ACC
  acc.readAccel(&ax, &ay, &az); //read the accelerometer values and store them in variables  x,y,z
  rawX = ax - aoffsetX;
  rawY = ay - aoffsetY;
  rawZ = az  - aoffsetZ + 256;
  X = rawX / 256.00; // used for angle calculations
  Y = rawY / 256.00; // used for angle calculations
  Z = rawZ / 256.00; // used for angle calculations
  rolldeg = 180 * (atan2(Y, Z)) / PI; // calculated angle in degrees
  pitchdeg = 180 * (atan2(-X, sqrt(Y * Y + Z * Z))) / PI; // calculated angle in degrees

  //Mag
  norm = compass.readNormalize();

  //deal with tilt
  float GN[3] = {X, Y, Z};
  float MG[3] = {norm.XAxis, norm.YAxis, norm.ZAxis};
  float GX[3] = {100, 0, 0};
  float M_project[3];
  float GX_project[3];
  projectN(MG, GN, M_project);
  projectN(GX, GN, GX_project);
  yawdeg = computeYaw(M_project, GX_project, GN);

  //tilt
  yawrad2 = -atan2(norm.YAxis, norm.XAxis);
  yawrad2 += declinationAngle;
  yawdeg2 = yawrad2 * 180 / M_PI;

}

void getRrate() {
  gyro.readGyro(&gx, &gy, &gz);
  gx_rateR = (gx - goffsetX) / 14.375;
  gy_rateR = (gy - goffsetY) / 14.375;
  gz_rateR = (gz - goffsetZ) / 14.375;

  gx_rate = (gx_rateR + gx_rateL) / 2;
  gy_rate = (gy_rateR + gy_rateL) / 2;
  gz_rate = (gz_rateR + gz_rateL) / 2;

  gx_rateL = gx_rateR;
  gy_rateL = gy_rateR;
  gz_rateL = gz_rateR;
}

//=============================================================================

void setup() {

  Serial.begin(9600);

  setupSensor();

  time = millis();
}


void loop() {

  getPose();

//  float a = rolldeg/180.*PI;
//  float b = pitchdeg/180.*PI;
//  float c = yawdeg/180.*PI;

//  z[0] = cos(a/2)*cos(b/2)*cos(c/2) + sin(a/2)*sin(b/2)*sin(c/2);
//  z[1] = sin(a/2)*cos(b/2)*cos(c/2) - cos(a/2)*sin(b/2)*sin(c/2);
//  z[2] = cos(a/2)*sin(b/2)*cos(c/2) + sin(a/2)*cos(b/2)*sin(c/2);
//  z[3] = cos(a/2)*cos(b/2)*sin(c/2) - sin(a/2)*sin(b/2)*cos(c/2);

  z[0] = rolldeg;
  z[1] = pitchdeg;
  z[2] = yawdeg;

  looptime = millis() - time;
  time = millis();

  getRrate();
  ekf.step(z);

  float r = ekf.getX(0) / 180.f * PI;
  float p = ekf.getX(1) / 180.f * PI;
  float y = ekf.getX(2) / 180.f * PI;

  float m00=cos(p)*cos(y),                     m01 = cos(p)*sin(y),                     m02 = -sin(p);
  float m10=sin(r)*sin(p)*cos(y)-cos(r)*sin(y),m11 = sin(r)*sin(p)*sin(y)+cos(r)*cos(y),m12 = sin(r)*cos(p);
  float m20=cos(r)*sin(p)*cos(y)+sin(r)*sin(y),m21 = cos(r)*sin(p)*sin(y)-sin(r)*cos(y),m22 = cos(r)*cos(p);

//  Serial.print(ekf.getX(0));
//  Serial.print(",\t");
//  Serial.print(ekf.getX(1));
//  Serial.print(",\t");
//  Serial.print(ekf.getX(2));
//  //Serial.print(",\t");
//  //Serial.print(ekf.getX(3));
//  Serial.print(",\t");
//  Serial.print(z[0]);
//  Serial.print(",\t");
//  Serial.print(z[1]);
//  Serial.print(",\t");
//  Serial.println(z[2]);
//  //Serial.print(",\t");
//  //Serial.println(z[3]);

  Serial.print(m00);
  Serial.print(" ");
  Serial.print(m10);
  Serial.print(" ");
  Serial.print(m20);
  Serial.print(" ");
  Serial.print(m01);
  Serial.print(" ");
  Serial.print(m11);
  Serial.print(" ");
  Serial.print(m21);
  Serial.print(" ");
  Serial.print(m02);
  Serial.print(" ");
  Serial.print(m12);
  Serial.print(" ");
  Serial.println(m22);
  
}
