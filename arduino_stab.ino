
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include <Servo.h>
#include "pid.h"
#include "params.h"


float zeroAngle(float angle, float zero) {
  if (fabs(angle - zero)  <= 180) {
    return angle - zero;
  }
  else {
    return 360 - fabs(angle - zero);
  }
}


#define RAD_TO_DEG 57.295779513

float roll_a = 0;
float pitch_a = 0;

unsigned long prev_t = 0;
unsigned long d_t = 0;

Servo roll_servo;
Servo pitch_servo;
Servo pitch_servo2;

PID roll_pid;
PID pitch_pid;

PID roll_pid_rate;
PID pitch_pid_rate;


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

float roll_mef_a[8] = { -1983233334, -1983233334, -1983233334, -1983233334, 
//-1983233334, -1983233334, -1983233334, -1983233334, 
-1983233334, -1983233334, -1983233334, -1983233334};
float pitch_mef_a[8] = { -1983233334, -1983233334, -1983233334, -1983233334, 
//-1983233334, -1983233334, -1983233334, -1983233334, 
-1983233334, -1983233334, -1983233334, -1983233334};
int ssss = 8;

float med_r_f(float value)
{
  for (int i = ssss - 1; i > 0; i--)
  {
    roll_mef_a[i] = roll_mef_a[i - 1];
  }
  roll_mef_a[0] = value;
  float cl = 0;
  int proc_d = 0;
  for (int i = 0; i < ssss; i++)
  {
    if (roll_mef_a[i] == -1983233334)
    {
      continue;
    }
    else
    {
      proc_d++;
      cl += roll_mef_a[i];
      //      float pv = 0;
      //      pv = arr[i];
      //      arr[i] = arr[i+1];
    }
  }
  if (proc_d == 0)
  {
    return value;
  }
  else
  {
    cl /= proc_d;
    return cl;
  }
}

float med_p_f(float value)
{
  for (int i = ssss - 1; i > 0; i--)
  {
    pitch_mef_a[i] = pitch_mef_a[i - 1];
  }
  pitch_mef_a[0] = value;
  float cl = 0;
  int proc_d = 0;
  for (int i = 0; i < ssss; i++)
  {
    if (pitch_mef_a[i] == -1983233334)
    {
      continue;
    }
    else
    {
      proc_d++;
      cl += pitch_mef_a[i];
      //      float pv = 0;
      //      pv = arr[i];
      //      arr[i] = arr[i+1];
    }
  }
  if (proc_d == 0)
  {
    return value;
  }
  else
  {
    cl /= proc_d;
    return cl;
  }
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  Serial.begin(115200);
  roll_servo.attach(ROLL_PIN);
  pitch_servo.attach(PITCH_PIN);
  pitch_servo2.attach(PITCH_PIN2);

  roll_servo.write(ROLL_S);
  pitch_servo.write(PITCH_S);
  pitch_servo2.write(PITCH_S2);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));


  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(-5);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  roll_pid.kP = ROLL_P;
  roll_pid.kI = ROLL_I;
  roll_pid.kD = ROLL_D;

  pitch_pid.kP = PITCH_P;
  pitch_pid.kI = PITCH_I;
  pitch_pid.kD = PITCH_D;

  roll_pid_rate.kP = ROLL_P_RATE;
  roll_pid_rate.kI = ROLL_I_RATE;
  roll_pid_rate.kD = ROLL_D_RATE;

  pitch_pid_rate.kP = PITCH_P_RATE;
  pitch_pid_rate.kI = PITCH_I_RATE;
  pitch_pid_rate.kD = PITCH_D_RATE;


  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  prev_t = millis();
}

float roll_ra = 0;
float pitch_ra = 0;

float roll_prev = 0;
float pitch_prev = 0;
//float roll_a_prev = 0;
//float pitch_prev = 0;
//unsigned long iter = 0;
float roll_s_prev = 0;
float pitch_s_prev = 0;
//float roll_a_prev = 0;
//float pitch_a_prev = 0;
bool start = false;

void loop() {

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }

  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    //    iter += 1;
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (((millis() - prev_t) > 3000) && !start) {
      start = true;

    }
    else {
      float roll = ypr[2] * RAD_TO_DEG;
      float pitch = ypr[1] * RAD_TO_DEG;
      roll = zeroAngle(roll, ROLL_ZERO);
      pitch = -zeroAngle(pitch, PITCH_ZERO);
      
      roll = zeroAngle(roll, ROLL_ZERO2);
      pitch = zeroAngle(pitch, PITCH_ZERO2);

      roll = med_r_f(roll)*ROLL_MED_F_C+roll*(1-ROLL_MED_F_C);
      pitch = med_p_f(pitch)*PITCH_MED_F_C+pitch*(1-PITCH_MED_F_C);

      unsigned long now = millis();
      d_t = now - prev_t;

      roll_ra = (roll - roll_prev) / ((float)d_t / 1000.0);
      pitch_ra = (pitch - pitch_prev) / ((float)d_t / 1000.0);

      roll_prev = roll;
      pitch_prev = pitch;

      //    float roll_s = roll_pid.calc(roll, 0);
      //    float pitch_s = pitch_pid.calc(pitch, 0);
      //    roll_a = roll_s;
      //    pitch_a = pitch_s;
      //    roll_a =
      //    float roll_r = roll + (roll_a - ROLL_S);
      //    float pitch_r = pitch + (pitch_a - PITCH_S);

      //    float roll_s = roll_pid.calc(roll_r, 0);
      //    float pitch_s = pitch_pid.calc(pitch_r, 0);

      //    roll_servo.write(roll_s + ROLL_S);
      //    pitch_servo.write(pitch_s + PITCH_S);

      //      float roll_s = roll_pid.calc(roll_ra, 0);
      //      float pitch_s = pitch_pid.calc(pitch_ra, 0);
      //      roll_a += roll_s;
      //      pitch_a += pitch_s;

      float roll_s = roll_pid.calc(roll, 0);
      float pitch_s = pitch_pid.calc(pitch, 0);

      float roll_s_ra = roll_pid_rate.calc(roll_ra, 0);
      float pitch_s_ra = pitch_pid_rate.calc(pitch_ra, 0);

      
      
      
      if (abs(roll_s_prev - roll_s) > ROLL_STATE_TH){
        roll_s /= d_t;
        
        roll_a += roll_s;
      }
      if (abs(pitch_s_prev - pitch_s) > PITCH_STATE_TH){
        pitch_s /= d_t;
        
        pitch_a += pitch_s;
      }
      
//      pitch_a += pitch_s;

      roll_a += roll_s_ra;
      pitch_a += pitch_s_ra;
      
      roll_s_prev = roll_s;
      pitch_s_prev = pitch_s;


//      roll_servo.writeMicroseconds(map(ROLL_S + roll_a, 0, 180, 500, 2400));
      roll_servo.writeMicroseconds(map(ROLL_S + roll_a, 0, 180, 500, 2400));
      pitch_servo.writeMicroseconds(map(PITCH_S - pitch_a, 0, 180, 500, 2400));
      pitch_servo2.writeMicroseconds(map(PITCH_S2 + pitch_a, 0, 180, 500, 2400));


      //    roll_servo.write(RqOLL_S + roll_a);
      //    pitch_servo.write(PITCH_S-pitch_a);
      //    pitch_servo2.write(PITCH_S2+pitch_a);

      //    Serial.print("ypr\t");
      //    Serial.print(ypr[0] * 180 / M_PI);
      //    Serial.print("\t");
      //    Serial.print(ypr[1] * 180 / M_PI);
      //    Serial.print("\t");
      //    Serial.println(ypr[2] * 180 / M_PI);

      //    roll_servo.write(ROLL_S + roll_a);
      //    pitch_servo.write(PITCH_S - pitch_a);
      //    pitch_servo2.write(PITCH_S2 + pitch_a);
      //    microValue = map(angleDegrees, 0,180,1000,2000);

      float p = 70;
      if (roll_a > p) {
        roll_a = p;
      }
      if (pitch_a > p) {
        pitch_a = p;
      }
      if (roll_a < -p) {
        roll_a = -p;
      }
      if (pitch_a < -p) {
        pitch_a = -p;
      }
      String debug_str = "";
      debug_str += String("roll: ") + String(roll) + String("\tpitch: ") + String(pitch);
      debug_str += String("\troll_rate: ") + String(roll_ra) + String("\tpitch_rate: ") + String(pitch_ra);
      debug_str += String("\troll_a: ") + String(roll_a) + String("\tpitch_a: ") + String(pitch_a);
      Serial.println(debug_str);


      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
            delay(1);
      prev_t = now;
      
    }
  }
}
