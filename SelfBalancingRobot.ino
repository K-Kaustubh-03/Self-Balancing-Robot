#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

#define MPU6050_ADDRESS 0x68

double Setpoint=0, Input, Output;
double Kp = 17, Ki = 0, Kd = 6; 
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

MPU6050 mpu;


const float ACCEL_SCALE_FACTOR = 9.80665 / 16384.0; 
const float GYRO_SCALE_FACTOR = 250.0 / 32768.0; 

// First Motor connections
int ENA = 10;
int IN1 = 9;
int IN2 = 8;
// Second Motor connections
int ENB = 5;
int IN3 = 6;
int IN4 = 7;


const int numReadings = 10;
float readings[numReadings];


void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // mpu.CalibrateAccel();
  // mpu.CalibrateGyro();

  //mpu.setXAccelOffset(-6051);
  //mpu.setYAccelOffset(-1442);
  //mpu.setZAccelOffset(1302);
  //mpu.setXGyroOffset(-2);
  //mpu.setYGyroOffset(-72);
  //mpu.setZGyroOffset(3);

  // Initialize PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

}

void loop() {

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax * ACCEL_SCALE_FACTOR; 
  float accelY = ay * ACCEL_SCALE_FACTOR; 
  float accelZ = az * ACCEL_SCALE_FACTOR; 
  float gyroX = gx * GYRO_SCALE_FACTOR; 
  float gyroY = gy * GYRO_SCALE_FACTOR; 
  float gyroZ = gz * GYRO_SCALE_FACTOR; 

  // Serial.print(accelX); Serial.print(" ");
  // Serial.print(accelY); Serial.print(" ");
  // Serial.print(accelZ); Serial.print(" ");
  // Serial.print(gyroX); Serial.print(" ");
  // Serial.print(gyroY); Serial.print(" ");
  // Serial.print(gyroZ); Serial.print(" ");


  // Pitch angle
  double angle = atan2(-accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / M_PI;
  for(int i = 0; i < numReadings-1; i++){
    readings[i] = readings[i+1];
  }
  readings[numReadings-1]=angle;

  float total = 0;
  for (int i = 0; i < numReadings; ++i){
    total+=readings[i];
  }
  double smoothedAngle = total / numReadings;
  // if(smoothedAngle<0)   smoothedAngle*=2;

  Input = smoothedAngle;

  pid.Compute();
  // if(smoothedAngle<0 && Output<128){
  //   Output*=2;
  // }  

  if (abs(Output) > 0.1) {
    if (Output > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

    }
    analogWrite(ENA, abs(Output));
    analogWrite(ENB, abs(Output));
  } else {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  }

  Serial.print("Angle: \t");
  Serial.print(angle);
  Serial.print("\tSmoothedAngle: \t");
  Serial.print(smoothedAngle);
  Serial.print("\t Output: ");
  Serial.println(Output);

  delay(1);
}
