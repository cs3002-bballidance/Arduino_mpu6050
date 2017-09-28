// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// https://www.i2cdevlib.com/forums/topic/12-a-question-to-change-the-sample-rate-in-mpu6050h-file/

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

#define SAMPLE_RATE 50  // 50hz sampling rate
#define DLPF_MODE 6
#define ACC1 8          // AD0 pin connect for accelerometer 1
#define ACC2 9          // AD0 pin connect for accelerometer 2
#define GYRO 10         // AD0 pin connect for gyro

// class default I2C address is 0x68
MPU6050 mpu; // default 0x68 i2c address on AD0 low 

const int samplemillis = 1000/SAMPLE_RATE; // duration between each sample, 20ms = 50Hz sampling rate
long currmillis = 0;
long startmillis = 0;

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
    
    // initialize device
    mpuselect(ACC1);
    mpu.initialize();
    mpu.setRate(SAMPLE_RATE);                     //set rate to 50Hz for sampling
    mpu.setDLPFMode(DLPF_MODE);                   //set on-board digital low-pass filter configuration
    
    mpuselect(ACC2);
    mpu.initialize();
    mpu.setRate(SAMPLE_RATE);                     //set rate to 50Hz for sampling
    mpu.setDLPFMode(DLPF_MODE);                   //set on-board digital low-pass filter configuration
    
    mpuselect(GYRO);
    mpu.initialize();
    mpu.setRate(SAMPLE_RATE);                     //set rate to 50Hz for sampling
    mpu.setDLPFMode(DLPF_MODE);                   //set on-board digital low-pass filter configuration
    
    // verify connection
    Serial.println("Testing MPU connection...");
    mpuselect(ACC1);
    Serial.println(mpu.testConnection() ? "MPU6050(acc1) connection successful" : "MPU6050(acc1) connection failed");
    mpuselect(ACC2);
    Serial.println(mpu.testConnection() ? "MPU6050(acc2) connection successful" : "MPU6050(acc1) connection failed");
    mpuselect(GYRO);
    Serial.println(mpu.testConnection() ? "MPU6050(gyro) connection successful" : "MPU6050(acc1) connection failed");

//    mpuselect(ACC1);
//    Serial.print("(3)X offset: ");
//    Serial.println(mpu.getXAccelOffset()); // or mpu.getXGyroOffsetTC()
//    Serial.print("(3)Y Offset: ");
//    Serial.println(mpu.getYAccelOffset());
//    Serial.print("(3)Z Offset: ");
//    Serial.println(mpu.getZAccelOffset());

    mpuselect(ACC1);
    mpu.setXAccelOffset(-4489);
    mpu.setYAccelOffset(1396);
    mpu.setZAccelOffset(400);
    mpu.setXGyroOffset(-76);
    mpu.setYGyroOffset(-42);
    mpu.setZGyroOffset(262);

    mpuselect(ACC2);
    mpu.setXAccelOffset(-1536);
    mpu.setYAccelOffset(-586);
    mpu.setZAccelOffset(1161);
    mpu.setXGyroOffset(-1268);
    mpu.setYGyroOffset(-19);
    mpu.setZGyroOffset(15);
    
    mpuselect(GYRO);
    mpu.setXGyroOffset(-238);
    mpu.setYGyroOffset(2293);
    mpu.setZGyroOffset(2215);
    mpu.setXGyroOffset(90);
    mpu.setYGyroOffset(30);
    mpu.setZGyroOffset(34);
        
    delay(3000);
    
    // use the code below to change accel/gyro offset values    
//    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//    Serial.print("\n");
//    accelgyro.setXGyroOffset(220);
//    accelgyro.setYGyroOffset(76);
//    accelgyro.setZGyroOffset(-85);
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//    Serial.print("\n");
    
    startmillis = millis();
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
      // these methods (and a few others) are also available
      mpuselect(ACC1);
      mpu.getAcceleration(&ax, &ay, &az);
      mpuselect(ACC2);
      mpu.getAcceleration(&ax2, &ay2, &az2);
      mpuselect(GYRO);
      mpu.getRotation(&gx, &gy, &gz);

      // Output in readeable format. Slow
//      Serial.print("1"); Serial.print("\t");
//      Serial.print(ax); Serial.print("\t");
//      Serial.print(ay); Serial.print("\t");
//      Serial.println(az);

//      Serial.print("2"); Serial.print("\t");
//      Serial.print(ax2); Serial.print("\t");
//      Serial.print(ay2); Serial.print("\t");
//      Serial.println(az2);

//      Serial.print("3"); Serial.print("\t");
//      Serial.print(gx); Serial.print("\t");
//      Serial.print(gy); Serial.print("\t");
//      Serial.println(gz);

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
