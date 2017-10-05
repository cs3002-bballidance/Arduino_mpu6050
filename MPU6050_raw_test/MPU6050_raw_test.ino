// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// https://www.i2cdevlib.com/forums/topic/12-a-question-to-change-the-sample-rate-in-mpu6050h-file/
// Useful site: http://samselectronicsprojects.blogspot.sg/2014/07/processing-data-from-mpu-6050.html?m=1
// http://cache.freescale.com/files/sensors/doc/app_note/AN3447.pdf
// https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define SAMPLE_RATE 40  // 50hz sampling rate
#define DLPF_MODE   0   // digital low pass filter mode
#define ACC1        8   // AD0 pin connect for accelerometer 1
#define ACC2        9   // AD0 pin connect for accelerometer 2
#define GYRO        1   // AD0 pin connect for gyro

// class default I2C address is 0x68
MPU6050 mpu; // default 0x68 i2c address on AD0 low 

const int samplemillis = 1000/SAMPLE_RATE; // duration between each sample, 20ms = 50Hz sampling rate

long currmillis = 0;
long startmillis = 0;

int avgAcc1X = 0;
int avgAcc1Y = 0;
int avgAcc1Z = 0;
int avgAcc2X = 0;
int avgAcc2Y = 0;
int avgAcc2Z = 0;

const int numSample = 100;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);   // set I2C speed to 400kbps. Max speed 500kbps
    #endif

    pinMode(ACC1, OUTPUT);
    pinMode(ACC2, OUTPUT);
    pinMode(GYRO, OUTPUT);
    digitalWrite(ACC1, HIGH);
    digitalWrite(ACC2, HIGH);
    digitalWrite(GYRO, HIGH);
        
    Serial.begin(115200); // usb serial

//    Serial.println("Initializing sensors...");
    // initialize device
    setSensors(ACC1, SAMPLE_RATE);
    setSensors(ACC2, SAMPLE_RATE);
    setSensors(GYRO, SAMPLE_RATE);

    setOffset(ACC1, -4516, 1386, 389, -67, -44, 258);   // calibrated offset for each sensors
    setOffset(ACC2, -1568, -590, 1163, -1257, -26, 15);
    setOffset(GYRO, -240, 2307, 2200, 89, 30, 45);
    
    // verify connection
//    Serial.println("Testing MPU connection...");
//    mpuselect(ACC1);
//    Serial.println(mpu.testConnection() ? "MPU6050(acc1) connection successful" : "MPU6050(acc1) connection failed");
//    mpuselect(ACC2);
//    Serial.println(mpu.testConnection() ? "MPU6050(acc2) connection successful" : "MPU6050(acc1) connection failed");
//    mpuselect(GYRO);
//    Serial.println(mpu.testConnection() ? "MPU6050(gyro) connection successful" : "MPU6050(acc1) connection failed");

    calibrateInitial();
    delay(3000);
    
    Serial.println("Start polling...");
    startmillis = millis();
}

void setSensors(int mpuNum, int sampleRate){
  mpuselect(mpuNum);
  mpu.initialize();
  mpu.setRate(sampleRate);                     //set rate to 50Hz for sampling
  mpu.setDLPFMode(DLPF_MODE);                   //set on-board digital low-pass filter configuration  
  //mpu.setFullScaleAccelRange(1);
    /*
   * AFS_SEL | Full Scale Range   | LSB Sensitivity
   * --------+--------------------+----------------
   * 0       |       +/- 2g       | 16384 LSB/g
   * 1       |       +/- 4g       | 8192 LSB/g
   * 2       |       +/- 8g       | 4096 LSB/g
   * 3       |       +/- 16g      | 2048 LSB/g
  */
  //mpu.setFullScaleGyroRange(0);
//  Serial.print("Acc range: "); Serial.println(mpu.getFullScaleAccelRange());
//  Serial.print("Gyro range: "); Serial.println(mpu.getFullScaleGyroRange());
  /*
   * FS_SEL | Full Scale Range   | LSB Sensitivity
   * -------+--------------------+----------------
   * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
   * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
   * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
   * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
  */
}

void setOffset(int mpuNum, int accX, int  accY, int accZ, int gyroX, int gyroY, int gyroZ){
  mpuselect(mpuNum);
  mpu.setXAccelOffset(accX);
  mpu.setYAccelOffset(accY);
  mpu.setZAccelOffset(accZ);
  mpu.setXGyroOffset(gyroX);
  mpu.setYGyroOffset(gyroY);
  mpu.setZGyroOffset(gyroZ);
}

void calibrateInitial(){
  int16_t ax, ay, az;
  int16_t ax2, ay2, az2;

  float sumAcc1X = 0;
  float sumAcc1Y = 0;
  float sumAcc1Z = 0;
  float sumAcc2X = 0;
  float sumAcc2Y = 0;
  float sumAcc2Z = 0;

  for(int i = 0; i < numSample; i++){
    mpuselect(ACC1);
    mpu.getAcceleration(&ax, &ay, &az);
    mpuselect(ACC2);
    mpu.getAcceleration(&ax2, &ay2, &az2);
    sumAcc1X += ax;
    sumAcc1Y += ay;
    sumAcc1Z += az;
    sumAcc2X += ax2;
    sumAcc2Y += ay2;
    sumAcc2Z += az2;
    delay(20);
  }
  avgAcc1X = (int)(sumAcc1X / numSample);
  avgAcc1Y = (int)(sumAcc1Y / numSample);
  avgAcc1Z = (int)(sumAcc1Z / numSample);
  avgAcc2X = (int)(sumAcc2X / numSample);
  avgAcc2Y = (int)(sumAcc2Y / numSample);
  avgAcc2Z = (int)(sumAcc2Z / numSample);
}

void mpuselect(int numMpu){
  switch(numMpu){
    case ACC1:
      digitalWrite(ACC1, LOW);
      digitalWrite(ACC2, HIGH);
      digitalWrite(GYRO, HIGH);
      break;
    case ACC2:
      digitalWrite(ACC1, HIGH);
      digitalWrite(ACC2, LOW);
      digitalWrite(GYRO, HIGH);
      break;
    case GYRO:
      digitalWrite(ACC1, HIGH);
      digitalWrite(ACC2, HIGH);
      digitalWrite(GYRO, LOW);
      break;
  }
}

void loop() {
    int16_t ax, ay, az;
    int16_t ax2, ay2, az2;
    int16_t gx, gy, gz;
    
    currmillis = millis();
    if ((currmillis - startmillis) >= samplemillis){
      mpuselect(ACC1);
      mpu.getAcceleration(&ax, &ay, &az);
      mpuselect(ACC2);
      mpu.getAcceleration(&ax2, &ay2, &az2);
      mpuselect(GYRO);
      mpu.getRotation(&gx, &gy, &gz);

      // Output in readeable format. Slow
//      Serial.print("1"); Serial.print("\t");
      Serial.print((int)(((ax-avgAcc1X)/8192.0)*9.81)); Serial.print(",");
      Serial.print((int)(((ay-avgAcc1Y)/8192.0)*9.81)); Serial.print(",");
      Serial.print((int)(((az-avgAcc1Z)/8192.0)*9.81)); Serial.print(",");

//      Serial.print("2"); Serial.print("\t");
      Serial.print((int)(((ax2-avgAcc2X)/8192.0)*9.81)); Serial.print(",");
      Serial.print((int)(((ay2-avgAcc2Y)/8192.0)*9.81)); Serial.print(",");
      Serial.print((int)(((az2-avgAcc2Z)/8192.0)*9.81)); Serial.print(",");

//      Serial.print("3"); Serial.print("\t");
      Serial.print((int)((gx)/250.0)); Serial.print(",");
      Serial.print((int)((gy)/250.0)); Serial.print(",");
      Serial.println((int)((gz)/250.0));

      // To output in binary (fast, uncompressed and no data loss), use the following:
//      Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));   // acc1 x-axis
//      Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));   // acc1 y-axis
//      Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));   // acc1 z-axis

//      Serial.write((uint8_t)(ax2 >> 8)); Serial.write((uint8_t)(ax2 & 0xFF));   // acc2 x-axis
//      Serial.write((uint8_t)(ay2 >> 8)); Serial.write((uint8_t)(ay2 & 0xFF));   // acc2 y-axis
//      Serial.write((uint8_t)(az2 >> 8)); Serial.write((uint8_t)(az2 & 0xFF));   // acc2 z-axis

//      Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));   // gyro x-axis
//      Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));   // gyro y-axis
//      Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));   // gyro z-axis
      startmillis = millis();
    }
}
