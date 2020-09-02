#include <Wire.h>
#include <Servo.h>
#define SerialPort Serial
#include <SparkFunMPU9250-DMP.h>
MPU9250_DMP imu;
double roll , pitch;
float phi = 0;
float theta = 0;
long int pre_ts=0;
float phi_n_1, theta_n_1;
float phi_dot, theta_dot, phi_quat;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float Q[4] = {1,0,0,0} ; 
  
float Q_dot [4] ;
float Q_pre [4] = {1,0,0,0}  ;







void setup()
{

{
  {
  SerialPort.begin(9600);

 if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(3000);
    }
  }

 
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

 
  imu.setGyroFSR(250); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(10); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(50); // Set mag rate to 10Hz
}
//Wire.begin();
//for (int cal_int = 0; cal_int < 1000 ; cal_int ++){ 
  //read_IMUDATA();
  pre_ts=millis();
}

}
void loop() 
{
    if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData(millis()-pre_ts);
    pre_ts=millis();
  }
}

void printIMUData(long int dt)
{  
  
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx)/57.3;
  float gyroY = imu.calcGyro(imu.gy)/57.3;
  float gyroZ = imu.calcGyro(imu.gz)/57.3;


// NORMALIZE ACCELE VALUES
 float naccel = sqrt(accelX*accelX +accelY*accelY+accelZ*accelZ);
   
    accelX =accelX/naccel;
    accelY =accelY/naccel;
    accelZ =accelZ/naccel;



  //Euler angle from accel
 
   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

 
    

   roll =  (0.98*(roll+(gyroX)*dt/1000.0f) +0.02*(accelX))*57.3;
   pitch = (0.98*(pitch + gyroY*dt/1000.0f)+0.02*(accelY))*57.3;


 

// quaternions

 Q_dot[0] = -0.5* ( (gyroX*Q_pre[1]) +(gyroY*Q_pre[2]) + (Q_pre[3]*gyroZ));
  Q_dot[1] = 0.5* ( (gyroX*Q_pre[0]) +(gyroZ*Q_pre[2]) - (Q_pre[3]*gyroY));
   Q_dot[2] = 0.5* ( (gyroY*Q_pre[0]) -(gyroZ*Q_pre[1]) + (Q_pre[3]*gyroX));
    Q_dot[3] = 0.5* ( (gyroZ*Q_pre[0]) +(gyroY*Q_pre[1]) - (Q_pre[2]*gyroX));

Q[0] =Q_pre[0]+ (Q_dot[0]*dt/1000.0);
Q_pre[0] = Q[0];
Q[1] =Q_pre[1]+ (Q_dot[1]*dt/1000.0);
Q_pre[1] = Q[1];
Q[2] =Q_pre[2]+ (Q_dot[2]*dt/1000.0);
Q_pre[2] = Q[2];
Q[3] =Q_pre[3]+ (Q_dot[3]*dt/1000.0);
Q_pre[3] = Q[3];

 double n = (sqrt((Q[0]*Q[0]) + (Q[1]*Q[1]) + (Q[2]*Q[2]) + (Q[3]*Q[3])));
float Q0 =Q[0] / n;
float Q1 =Q[1] / n;
float Q2 = Q[2]/n;
float Q3 = Q[3]/n;

phi_quat = atan2 (2*((Q0*Q1)+(Q2*Q3)), ((0.5f-Q1*Q1-Q2*Q2)));
float theta_quat = asin( 2*((Q0*Q2)- (Q1*Q3)));

 
phi_quat = (0.98*(phi_quat+gyroX*dt/1000.0f) +0.02*(accelX))*57.3;
theta_quat = (0.98*(theta_quat+gyroY*dt/1000.0f) +0.02*(accelY))*57.3;

  

 int filtered_roll = 0.99*(roll+roll*dt/1000.0f)+0.01*(phi_quat);
 
 int filtered_pitch = 0.99*(pitch+pitch*dt/1000.0f)+0.01*(theta_quat);



 Serial.println(String(filtered_roll)+','+String(filtered_pitch));
  
   
    }
