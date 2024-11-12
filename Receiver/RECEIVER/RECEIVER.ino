
#include "CRSFforArduino.hpp"
#include <Servo.h>
#include <math.h>
#include <PID_v1.h>
#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

#define pitchPin PB3
#define yawPin PB5
#define ESCPin PB4 
#define lRollPin PB10
#define rRollPin PA8


HardwareSerial Serial1(PA10,PA9);
CRSFforArduino *crsf = nullptr;

int rcChannelCount = 8;

uint8_t sigmoid_LUT[181];

double compP=0;
double compR=0;

// double pidRoll_set, pidRoll_meas, pidRoll_out;
// double rollKp = 0.9, rollKi = 0.01, rollKd = 0.009;
// double pidPitch_set, pidPitch_meas, pidPitch_out;
// double pitchKp= 0.7, pitchKi= 0.01, pitchKd= 0.007; 
// PID rollPID(&compR, &pidRoll_out, &pidRoll_set, rollKp, rollKi, rollKd, DIRECT);
// PID pitchPID(&compP, &pidPitch_out, &pidPitch_set, pitchKp, pitchKi, pitchKd, DIRECT);

Servo pitch;
Servo Lroll;
Servo Rroll;
Servo yaw;
Servo ESC;

// MPU6050 mpu;
// KalmanFilter kalmanX(0.001, 0.003, 0.03);
// KalmanFilter kalmanY(0.001, 0.003, 0.03);
// float accPitch = 0;
// float accRoll = 0;

uint16_t auxVal[8];

const char *rcChannelNames[] = {
    "A",
    "E",
    "T",
    "R",
    "Aux1",
    "Aux2",
    "Aux3",
    "Aux4",

    "Aux5",
    "Aux6",
    "Aux7",
    "Aux8",
    "Aux9",
    "Aux10",
    "Aux11",
    "Aux12"};

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

uint8_t expoControl(uint8_t input, float expo){
  float normalized_input = (input-90)/90.0;
  float a = 1-expo;
  float normalized_output = a*normalized_input+expo*(pow(normalized_input,3));
  return normalized_output*90+90;
}

double control2deg(uint8_t input){
  return 45.0*(input-90)/90.0;

}

void setup()
{
    // Initialise the serial port & wait for the port to open.
    
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    pitch.attach(pitchPin);
    Lroll.attach(lRollPin);
    Rroll.attach(rRollPin);
    yaw.attach(yawPin);
    ESC.attach(ESCPin,1000,2000);
    digitalWrite(ESCPin,LOW);
    Serial.begin(115200);
    Serial.print("program start");
    
    // rollPID.SetMode(AUTOMATIC);
    // pitchPID.SetMode(AUTOMATIC);
    // rollPID.SetOutputLimits(-60, 60); 
    // pitchPID.SetOutputLimits(-60, 60);
    // while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    // {
    //   Serial.print("check point");
    //   delay(500); 
    // }
    // mpu.calibrateGyro();
    // while (!Serial)
    // {
    //     ;
    // }

    // Initialise CRSF for Arduino.
    crsf = new CRSFforArduino(&Serial1);
    if (!crsf->begin())
    {
        crsf->end();

        delete crsf;
        crsf = nullptr;

        Serial.println("CRSF for Arduino initialisation failed!");
        while (1)
        {
            delay(10);
        }
    }

    rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;

    crsf->setRcChannelsCallback(onReceiveRcChannels);
    // for(int i = 0; i<=180;i++){
    //   sigmoid_LUT[i] = 180.0/(1.0+exp((float(-i)+90.0)/20.0));
    // }
    Serial.println("Ready");
    delay(1000);
}

bool gyro_stable = false; 
double last_time = 0;
double dt;
void loop()
{
  crsf->update();
  // Vector acc = mpu.readNormalizeAccel();
  // Vector gyr = mpu.readNormalizeGyro();
  // // Calculate Pitch & Roll from accelerometer (deg)
  // accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  // accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // dt = (double)(micros()-last_time)/1000000;
  // last_time = micros();
  // compP = 0.99*(compP+gyr.YAxis*dt)+0.01*accPitch;
  // compR = 0.99*(compR+gyr.XAxis*dt)+0.01*accRoll;
  // // Kalman filter
  // // Serial.print(compP);
  // // Serial.print(":");
  // Serial.println(gyr.YAxis);
  // pidPitch_meas = kalmanY.update(accPitch, gyr.YAxis);
  // pidRoll_meas = kalmanX.update(accRoll, gyr.XAxis);

  // Serial.println(pidRoll_meas);
  // Serial.println(pidPitch_meas);

  // if(gyro_stable){
  //   rollPID.Compute();
  //   pitchPID.Compute();
  //   pitch.write(180-(90+pidPitch_out));
  //   Lroll.write(90+pidRoll_out); 
  //   Rroll.write(90+pidRoll_out);
  // }
}

bool motor_EN = false;
bool gyro_cali = false;
 

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
  if (rcChannels->failsafe == false){
    for (int i = 1; i <= rcChannelCount; i++)
    {
      auxVal[i-1] = map(crsf->getChannel(i),CRSF_RC_CHANNEL_MIN,
      CRSF_RC_CHANNEL_MAX, 0, 180);
      // auxVal[i-1] = sigmoid_LUT[auxVal[i-1]];
    }
  

  motor_EN = auxVal[4]>90;
  gyro_stable = auxVal[7]>90;
  gyro_cali = auxVal[6]>90;

  if(!gyro_stable){
    int roll_val = expoControl(auxVal[0],0.4);
    roll_val = map(roll_val,0,180,70,110);

    int pitch_val = expoControl(auxVal[1],0.6);
    pitch_val = map(pitch_val,0,180,65,115);

    pitch.write(180-pitch_val);
    Lroll.write(roll_val); 
    Rroll.write(roll_val);

  }
  else{
    if(gyro_cali){
      // mpu.calibrateGyro();
      // delay(500);
    }
    // pidPitch_set = control2deg(auxVal[1]);
    // pidRoll_set = control2deg(auxVal[0]);
    // Serial.println(pidRoll_set);

    
  }

  int yaw_val = expoControl(auxVal[3],0.2);
  yaw_val = map(yaw_val,0,180,70,110);

  int thr_val = map(auxVal[2],0,180,0,180);

  yaw.write(yaw_val);

  if(motor_EN){
    ESC.write(thr_val);
  }
  else{
    ESC.write(0);
  }
  
  }
}
