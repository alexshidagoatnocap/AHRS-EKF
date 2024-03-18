// Written by Alexander Shi
// Inspired by https://sikhrobotics.com/orientation/ekf/implementation/

#include <eigen.h>
#include <Eigen/LU>
#include <iostream>
#include <iomanip>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

// LSM6DSOX SPI Operation
#define LSM_CS 10 // CS on LSM6DSOX Breakout Board
#define LSM_SCK 13 // SCL on LSM6DSOX Breakout Board 
#define LSM_MISO 12 // DO on LSM6DSOX Breakout Board
#define LSM_MOSI 11 // SDA on LSM6DSOX Breakout Board

// LIS3MDL SPI Operation, for calibration, SCL goes to 19, SDA goes to 18
#define LIS3MDL_CLK 27 // SCL on LIS3MDL Breakout Board
#define LIS3MDL_MISO 1 // DO on LIS3MDL Breakout Board
#define LIS3MDL_MOSI 26 // SDA on LIS3MDL Breakout Board
#define LIS3MDL_CS 0 // CS on LIS3MDL Breakout Board

volatile byte accelDataRdy = false;
volatile byte magDataRdy = false;

const float GRAVITY = 9.81;
const float TIME_DELAY = 0.004;

float accPhi, accTheta, magPsi;
float magCalX, magCalY, magCalZ;

Eigen::MatrixXf x_k(7,1);
Eigen::MatrixXf P_k(7,7);
Eigen::MatrixXf   Q(7,7);
Eigen::MatrixXf   R(6,6);

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL mag;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX test!");

  //if (!sox.begin_I2C()) {
    // if (!sox.begin_SPI(LSM_CS)) {
  if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }

  // Search for LIS3MDL SPI
  //if (! lis3mdl.begin_I2C()) {
  if (! mag.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }

  Serial.println("LSM6DSOX Found!");

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  sox.setAccelDataRate(LSM6DS_RATE_52_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Mag performance mode set to: ");
  switch (mag.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Mag operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (mag.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  mag.setDataRate(LIS3MDL_DATARATE_40_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Mag data rate set to: ");
  switch (mag.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  mag.setRange(LIS3MDL_RANGE_16_GAUSS);
  Serial.print("Mag range set to: ");
  switch (mag.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  mag.setIntThreshold(500);
  mag.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  // Configure interrupts for data-ready
  sox.configInt2(false, false, true);
  pinMode(22, INPUT); // Accel interrupt
  attachInterrupt(digitalPinToInterrupt(22), accelDataReady, FALLING);
  pinMode(23, INPUT); // Mag interrupt
  attachInterrupt(digitalPinToInterrupt(23), magDataReady, FALLING);

  delay(200); // The delay is here because the magnetometer has bad readings upon startup
  
  // Initialize x_k and P_k
  EKFInit(sox, mag, x_k, P_k, Q, R);
  print_mtxf(x_k);
}

void loop() {

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t event;

  sox.getEvent(&accel, &gyro, &temp);
  mag.getEvent(&event);

  calibrateMag(event.magnetic.x, event.magnetic.y, event.magnetic.z, magCalX, magCalY, magCalZ);
  accelEuler(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, accPhi, accTheta);
  magHeading(magCalX, magCalY, magCalZ, magPsi);

  EKFPredict(x_k, P_k, Q, -gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
  
  if(accelDataRdy == HIGH) {
    EKFUpdate(x_k, P_k, Q, R, magCalX, magCalY, magCalZ, accel.acceleration.x, accel.acceleration.y, -accel.acceleration.z);
  } 

  if(magDataRdy == HIGH) {
    EKFUpdate(x_k, P_k, Q, R, magCalX, magCalY, magCalZ, accel.acceleration.x, accel.acceleration.y, -accel.acceleration.z);
  } 
  
  /*
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x, 7);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y, 7);
  Serial.print(" \tZ: ");
  Serial.print(-accel.acceleration.z, 7);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tGyro X: ");
  Serial.print(-gyro.gyro.x, 7);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y, 7);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z, 7);
  Serial.println(" radians/s ");

  Serial.print("\t\tMag X: "); Serial.print(-event.magnetic.y, 7);
  Serial.print(" \tY: "); Serial.print(event.magnetic.x, 7); 
  Serial.print(" \tZ: "); Serial.print(-event.magnetic.z, 7); 
  Serial.print(" uTesla ");
  Serial.println();
  */

  std::cout << std::fixed << std::setprecision(7);
  std::cout << "Measured Angles \t\tPhi: " << accPhi*RAD_TO_DEG;
  std::cout << "\tTheta: " << accTheta*RAD_TO_DEG;
  std::cout << "\tPsi: " << magPsi*RAD_TO_DEG;
  std::cout << " Degrees" << std::endl;

  std::cout << std::fixed << std::setprecision(3);
  std::cout << x_k(0,0) << std::endl << x_k(1,0) << std::endl << x_k(2,0) << std::endl << x_k(3,0) << std::endl << std::endl;
  quatToEuler(x_k);
  //pyTeapotQuat(x_k);

  accelDataRdy = false;
  magDataRdy = false;

  delay(TIME_DELAY*1000);
}

void pyTeapotQuat(const Eigen::MatrixXf& state)
{
  std::cout << "w" << state(0,0) << "wa" << state(1,0) << "ab" << state(2,0) << "bc" << state(3,0) << "c" << std::endl;
}

void accelDataReady(void)
{
  accelDataRdy = true;
}

void magDataReady(void)
{
  magDataRdy = true;
}

void print_mtxf(const Eigen::MatrixXf& X)  
{
   int i, j, nrow, ncol;
   nrow = X.rows();
   ncol = X.cols();
   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j), 6);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

void quatToEuler(Eigen::MatrixXf& state)
{
  float q0, q1, q2, q3;
  float phi, theta, psi;

  q0 = state(0,0);
  q1 = state(1,0);
  q2 = state(2,0);
  q3 = state(3,0);

  float phiCalc, psiCalc;

  phiCalc = (2*(q0*q1 + q2*q3))/(1 - 2*(q1*q1 + q2*q2));
  psiCalc = (2*(q0*q3 + q1*q2))/(1 - 2*(q2*q2 + q3*q3));

  phi = constrainAngle360(atan(phiCalc));
  theta = constrainAngle360(asin(2*(q0*q2 - q1*q3)));
  psi = constrainAngle360(atan(psiCalc));

  Serial.print("\nQuaternion to Euler: ");
  Serial.print("\t\tPhi ");
  Serial.print(phi*RAD_TO_DEG);

  Serial.print("\tTheta ");
  Serial.print(theta*RAD_TO_DEG);

  Serial.print("\tPsi ");
  Serial.println(psi*RAD_TO_DEG);
}

//Function copied from https://github.com/FlyTheThings/uNavINS/blob/master/uNavINS.cpp
float constrainAngle360(float dta) {
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}

void accelEuler(float accX, float accY, float accZ, float &accPhi, float &accTheta)
{
  //accPhi = atan(accY / -accZ);
  accPhi = constrainAngle360(atan2(-accY, accZ));
  accTheta = constrainAngle360(asin(-accX / sqrt(accX*accX + accY*accY + accZ*accZ)));
}

void calibrateMag(float rawX, float rawY, float rawZ, float &calX, float &calY, float &calZ)
{
  Eigen::MatrixXf hard_iron(3,1);
  hard_iron << -58.39,
               -17.18,  
               -74.22;

  Eigen::MatrixXf soft_iron(3,3);
  soft_iron << 0.945,  0.112, -0.062,
               0.112,  1.022, -0.004,
              -0.062, -0.004,  1.054;

  Eigen::MatrixXf rawValues(3,1);
  rawValues << rawX,
               rawY,
               rawZ;

  Eigen::MatrixXf calValues(3,1);
  calValues = soft_iron*(rawValues - hard_iron);
  
  calX = calValues(0,0);
  calY = calValues(1,0);
  calZ = calValues(2,0);
}

// Values must be calibrated first
void magHeading(float magX, float magY, float magZ, float &magPsi)
{
  const float mag_decl = 2.65;

  // Add tilt compensation later
  magPsi = constrainAngle360(atan2(magX, -magY) + mag_decl);
  //magPsi = map(magPsi, -0.5236, 5.76, 0, 2*PI);

  /*if (magPsi < 0.0) {
    magPsi += 360.0;
  }*/
}

void EKFInit(Adafruit_LSM6DSOX &sox, Adafruit_LIS3MDL &mag, Eigen::MatrixXf &x_0Plus, Eigen::MatrixXf &P_k, Eigen::MatrixXf &Q, Eigen::MatrixXf &R)
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t event;

  float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
  float accPhi, accTheta, magPsi;

  for(int i=0; i<14; i++) {
    sox.getEvent(&accel, &gyro, &temp);
    mag.getEvent(&event);

    calibrateMag(event.magnetic.x, event.magnetic.y, event.magnetic.z, magCalX, magCalY, magCalZ);
    accelEuler(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, accPhi, accTheta);
    magHeading(magCalX, magCalY, magCalZ, magPsi);

    q0 += cos(accPhi/2)*cos(accTheta/2)*cos(magPsi/2) + sin(accPhi/2)*sin(accTheta/2)*sin(magPsi/2);
    q1 += sin(accPhi/2)*cos(accTheta/2)*cos(magPsi/2) - cos(accPhi/2)*sin(accTheta/2)*sin(magPsi/2);
    q2 += cos(accPhi/2)*sin(accTheta/2)*cos(magPsi/2) + sin(accPhi/2)*cos(accTheta/2)*sin(magPsi/2);
    q3 += cos(accPhi/2)*cos(accTheta/2)*sin(magPsi/2) - sin(accPhi/2)*sin(accTheta/2)*cos(magPsi/2);
  }

  // Find Average of samples
  q0 = q0 / 15;
  q1 = q1 / 15;
  q2 = q2 / 15;
  q3 = q3 / 15;

  // Initialize x_k at the beginning, x_0Plus
  x_0Plus << q0,
             q1,
             q2,
             q3,
             0.0001,
             0.0001,
             0.0001;

  // Get one more event to calculate P_k
  sox.getEvent(&accel, &gyro, &temp);
  mag.getEvent(&event);

  calibrateMag(event.magnetic.x, event.magnetic.y, event.magnetic.z, magCalX, magCalY, magCalZ);
  accelEuler(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, accPhi, accTheta);

  q0 = cos(accPhi/2)*cos(accTheta/2)*cos(magPsi/2) + sin(accPhi/2)*sin(accTheta/2)*sin(magPsi/2);
  q1 = sin(accPhi/2)*cos(accTheta/2)*cos(magPsi/2) - cos(accPhi/2)*sin(accTheta/2)*sin(magPsi/2);
  q2 = cos(accPhi/2)*sin(accTheta/2)*cos(magPsi/2) + sin(accPhi/2)*cos(accTheta/2)*sin(magPsi/2);
  q3 = cos(accPhi/2)*cos(accTheta/2)*sin(magPsi/2) - sin(accPhi/2)*sin(accTheta/2)*cos(magPsi/2);

  Eigen::MatrixXf x_0(7,1);
  x_0 << q0,
         q1,
         q2,
         q3,
         0.0001,
         0.0001,
         0.0001;

  P_k = (x_0 - x_0Plus)*(x_0 - x_0Plus).transpose();
  
  // Initialize Q and R noise matrices
  Q << 1, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0, 0,
       0, 0, 0, 1, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 1;
  Q = 0.005*Q;
  
  R << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
  R = 0.005*R;

}

void EKFPredict(Eigen::MatrixXf &x_k, Eigen::MatrixXf &P_k, Eigen::MatrixXf &Q, float gyroX, float gyroY, float gyroZ)
{
  float q0, q1, q2, q3;
  q0 = x_k(0,0);
  q1 = x_k(1,0);
  q2 = x_k(2,0);
  q3 = x_k(3,0);
  
  Eigen::MatrixXf S(4,3);
  S << -q1, -q2, -q3,
        q0, -q3,  q2,
        q3,  q0, -q1,
       -q2,  q1,  q0;

  Eigen::MatrixXf S_A(4,3);
  S_A = (-TIME_DELAY/2)*S;

  Eigen::MatrixXf A(7,7);
  A << 1, 0, 0, 0, S_A(0,0), S_A(0,1), S_A(0,2),
       0, 1, 0, 0, S_A(1,0), S_A(1,1), S_A(1,2),
       0, 0, 1, 0, S_A(2,0), S_A(2,1), S_A(2,2),
       0, 0, 0, 1, S_A(3,0), S_A(3,1), S_A(3,2),
       0, 0, 0, 0,     1   ,     0   ,    0    ,
       0, 0, 0, 0,     0   ,     1   ,    0    ,
       0, 0, 0, 0,     0   ,     0   ,    1    ;

  Eigen::MatrixXf B(7,3);
  B << S(0,0), S(0,1), S(0,2),
       S(1,0), S(1,1), S(1,2),
       S(2,0), S(2,1), S(2,2),
       S(3,0), S(3,1), S(3,2),
          0  ,    0   ,   0  ,
          0  ,    0   ,   0  ,
          0  ,    0   ,   0  ;
  B = (TIME_DELAY/2)*B;

  // Stores gyro output values
  Eigen::MatrixXf u(3,1);
  u << gyroX,
       gyroY,
       gyroZ;

  // Get new prediction
  x_k = A*x_k + B*u;

  // Get new error covariance matrix
  P_k = A*P_k*A.transpose() + Q;
}

void EKFUpdate(Eigen::MatrixXf &x_k, Eigen::MatrixXf &P_k, Eigen::MatrixXf &Q, Eigen::MatrixXf &R, float magX, float magY, float magZ, float accX, float accY, float accZ)
{
  // Initialize state estimate matrix x_k variables
  double q0, q1, q2, q3;

  q0 = x_k(0,0);
  q1 = x_k(1,0);
  q2 = x_k(2,0);
  q3 = x_k(3,0);

  // Establish down and north reference vectors
  Eigen::MatrixXf V_dRef(3,1);
  V_dRef << 0,
            0,
           -1;
  
  //print_mtxf(V_dRef);
  
  Eigen::MatrixXf V_nRef(3,1);
  V_nRef <<   0,
              1,
              0;
  //print_mtxf(V_nRef);
  
  // Establish rotation matrix from NED to body frame or h(x_k)
  Eigen::MatrixXf R_nedToBody(3,3);
  R_nedToBody << 1-2*(q2*q2 + q3*q3), 2*(q1*q2 + q0*q3)  , 2*(q1*q3 - q0*q2)  ,
                 2*(q1*q2 - q0*q3)  , 1-2*(q1*q1 + q3*q3), 2*(q2*q3 + q0*q1)  ,
                 2*(q1*q3 + q0*q2)  , 2*(q2*q3 - q0*q1)  , 1-2*(q1*q1 + q2*q2);
  //Serial.println("\nNED To Body");
  //print_mtxf(R_nedToBody);

  // Body to NED frame rotation matrix
  Eigen::MatrixXf R_bodyToNED(3,3);
  R_bodyToNED << 1-2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3)  , 2*(q1*q3 + q0*q2)  ,
                 2*(q1*q2 + q0*q3)  , 1-2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)  ,
                 2*(q1*q3 - q0*q2)  , 2*(q2*q3 + q0*q1)  , 1-2*(q1*q1 + q2*q2);
  //Serial.println("\nBody To NED");
  //print_mtxf(R_bodyToNED);


  Eigen::MatrixXf h(3,3);
  h = R_nedToBody;
  
  // H is the jacobian of h, with h = R_nedToBody
  Eigen::MatrixXf H(6,7);
  H << 2*q2, -2*q3,  2*q0, -2*q1, 0, 0, 0,
      -2*q1, -2*q0, -2*q3, -2*q2, 0, 0, 0,
      -2*q0,  2*q1,  2*q2, -2*q3, 0, 0, 0,
       2*(q0*V_nRef(0,0) + q3*V_nRef(1,0)), 2*(q1*V_nRef(0,0) + q2*V_nRef(1,0)), 2*(-q2*V_nRef(0,0) + q1*V_nRef(1,0)), 2*(-q3*V_nRef(0,0) + q0*V_nRef(1,0)), 0, 0, 0,
       2*(-q3*V_nRef(0,0) + q0*V_nRef(1,0)),2*(q2*V_nRef(0,0) - q1*V_nRef(1,0)), 2*(q1*V_nRef(0,0) + q2*V_nRef(1,0)), 2*(-q0*V_nRef(0,0) - q3*V_nRef(1,0)), 0, 0, 0,
       2*(q2*V_nRef(0,0) - q1*V_nRef(1,0)), 2*(q3*V_nRef(0,0) - q0*V_nRef(1,0)), 2*(q0*V_nRef(0,0) + q3*V_nRef(1,0)), 2*(q1*V_nRef(0,0) + q2*V_nRef(1,0)), 0, 0, 0;
  //Serial.println("\nH Matrix");
  //print_mtxf(H);

  // Calculate Kalman Gain K
  Eigen::MatrixXf K(7,6);
  K = P_k*H.transpose() * (H*P_k*H.transpose() + R).inverse();
  //Serial.println("\nK Matrix");
  //print_mtxf(K);

  // Find accelerometer sensor prediction d_kMinusA
  Eigen::MatrixXf d_kMinus1A(3,1);
  d_kMinus1A = R_nedToBody * V_dRef;
  //Serial.println("\nd_kMinus1A Matrix");
  //print_mtxf(d_kMinus1A);

  // Find magnetometer sensor prediction d_kMinusM
  Eigen::MatrixXf d_kMinus1M(3,1);
  d_kMinus1M = R_nedToBody * V_nRef;
  //Serial.println("\nd_kMinus1M Matrix");
  //print_mtxf(d_kMinus1M);

  // Normalize both of the above vectors
  d_kMinus1A = d_kMinus1A / d_kMinus1A.norm();
  d_kMinus1M = d_kMinus1M / d_kMinus1M.norm();
  //Serial.println("\nd_kMinus1A Matrix Normed");
  //print_mtxf(d_kMinus1A);
  //Serial.println("\nd_kMinus1M Matrix Normed");
  //print_mtxf(d_kMinus1M);

  // Initialize measurement prediction matrix d_kMinus1
  Eigen::MatrixXf d_kMinus1(6,1);
  d_kMinus1(0,0) = d_kMinus1A(0,0);
  d_kMinus1(1,0) = d_kMinus1A(1,0);
  d_kMinus1(2,0) = d_kMinus1A(2,0);
  d_kMinus1(3,0) = d_kMinus1M(0,0);
  d_kMinus1(4,0) = d_kMinus1M(1,0);
  d_kMinus1(5,0) = d_kMinus1M(2,0);

  //Serial.println("\nd_kMinus1 Matrix");
  //print_mtxf(d_kMinus1);

  // Get measured magnetometer values to NED frame
  Eigen::MatrixXf z_magVal(3,1); // Stores mag data
  z_magVal(0,0) = magX;
  z_magVal(1,0) = magY;
  z_magVal(2,0) = magZ;
  
  //Serial.println("\nz_magVal Matrix");
  //print_mtxf(z_magVal);
  
  // Get in NED frame then set the magnetometer Z measurement to 0
  Eigen::MatrixXf z_mt(3,1);
  z_mt = R_bodyToNED*z_magVal;

  //Serial.println("\nz_mt Matrix");
  //print_mtxf(z_mt);

  // Get from NED to body
  Eigen::MatrixXf z_mf(3,1);
  z_mf = R_nedToBody * z_mt;
  z_mf(2,0) = 0.0;

  //Serial.println("\nz_mf Matrix");
  //print_mtxf(z_mf);

  // Set up z_k measurement matrix containing acc/mag outputs
  Eigen::MatrixXf z_k(6,1);
  
  // Normalize accelerometer outputs
  Eigen::MatrixXf z_a(3,1);
  z_a(0,0) = accX;
  z_a(1,0) = accY;
  z_a(2,0) = accZ;

  //Serial.println("\nz_a Matrix");
  //print_mtxf(z_a);
  
  z_a = z_a / z_a.norm();
  //Serial.println("\nz_a Matrix Normed");
  //print_mtxf(z_a);

  // Normalize magnetometer outputs
  z_mf = z_mf / z_mf.norm();

  //Serial.println("\nz_mf Matrix Normed");
  //print_mtxf(z_mf);

  // Assign measurement matrix z_k
  z_k(0,0) = z_a(0,0);
  z_k(1,0) = z_a(1,0);
  z_k(2,0) = z_a(2,0);
  z_k(3,0) = z_mf(0,0);
  z_k(4,0) = z_mf(1,0);
  z_k(5,0) = z_mf(2,0);

  //Serial.println("\nz_k Matrix");
  //print_mtxf(z_k);

  // Update state estimate
  x_k = x_k + K*(z_k - d_kMinus1);

  Serial.println("\nx_k Matrix Final");
  print_mtxf(x_k);

  // Update error covariance
  Eigen::MatrixXf I(7,7); //7x7 Identity because i'm still cant setup an identity matrix
  I << 1, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0, 0,
       0, 0, 0, 1, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 1;

  P_k = (I - K*H)*P_k;

  //Serial.println("\P_k Matrix Final");
  //print_mtxf(P_k);

  //Take updated state and calculate residual error
  R_nedToBody << 1-2*(q2*q2 + q3*q3), 2*(q1*q2 + q0*q3)  , 2*(q1*q3 - q0*q2)  ,
                 2*(q1*q2 - q0*q3)  , 1-2*(q1*q1 + q3*q3), 2*(q2*q3 + q0*q1)  ,
                 2*(q1*q3 + q0*q2)  , 2*(q2*q3 - q0*q1)  , 1-2*(q1*q1 + q2*q2);

  d_kMinus1A = R_nedToBody * V_nRef;
  d_kMinus1M = R_nedToBody * V_nRef;
  
  d_kMinus1A = d_kMinus1A / d_kMinus1A.norm();
  d_kMinus1M = d_kMinus1M / d_kMinus1M.norm();

  d_kMinus1(0,0) = d_kMinus1A(0,0);
  d_kMinus1(1,0) = d_kMinus1A(1,0);
  d_kMinus1(2,0) = d_kMinus1A(2,0);
  d_kMinus1(3,0) = d_kMinus1M(0,0);
  d_kMinus1(4,0) = d_kMinus1M(1,0);
  d_kMinus1(5,0) = d_kMinus1M(2,0);

  Eigen::MatrixXf e_k(6,1);
  e_k = z_k - d_kMinus1;

  //Get new tuning matrices
  float alpha = 0.5;

  //R = alpha*R + (1 - alpha)*(e_k*e_k.transpose() + H*P_k*H.transpose());
  //Q = alpha*Q + (1 - alpha)*(K*d_kMinus1*d_kMinus1.transpose()*K.transpose());

  //quatToEuler(x_k);
}