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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
MPU6050 mpu2(0x69); // <-- use for AD0 high
//MPU6050 gyro(0x71);

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#define ACC1AD0 53    // AD0 pin connect for accelerometer 1
#define ACC2AD0 52    // AD0 pin connect for accelerometer 2

const int samplemillis = 20; // duration between each sample, 20ms
long currmillis = 0;
long startmillis = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);   // set I2C speed to 400kbps
    #endif

    pinMode(ACC1AD0, OUTPUT);
    pinMode(ACC2AD0, OUTPUT);
    digitalWrite(ACC1AD0, LOW);   //0x68 to select ACC1AD0
    digitalWrite(ACC2AD0, HIGH);  //0x69 to select ACC2AD0
    
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();
    mpu2.initialize();
    mpu.setRate(50);                     //set rate to 100Hz for sampling
    mpu2.setRate(50);
    mpu.setDLPFMode(6);                   //set digital low-pass filter configuration
    mpu.setDLPFMode(6);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println(mpu2.testConnection() ? "MPU6050(2) connection successful" : "MPU6050(2) connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    //*/
    Serial.println("Starting now...");
    startmillis = millis();
}

void loop() {
    int16_t ax, ay, az;
    int16_t ax2, ay2, az2;
    //int16_t gx, gy, gz;
    
    // read raw accel/gyro measurements from device
//    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    currmillis = millis();
    if ((currmillis - startmillis) >= samplemillis){
      // these methods (and a few others) are also available
      mpu.getAcceleration(&ax, &ay, &az);
      mpu2.getAcceleration(&ax2, &ay2, &az2);
      //mpu.getRotation(&gx, &gy, &gz);
      Serial.print("1"); Serial.print("\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.println(az);
//      Serial.print("2"); Serial.print("\t");
//      Serial.print(ax2); Serial.print("\t");
//      Serial.print(ay2); Serial.print("\t");
//      Serial.println(az2);
      startmillis = millis();
    }

//    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
//        Serial.print("accel:\t");
//        Serial.print(ax); Serial.print("\t");
//        Serial.print(ay); Serial.print("\t");
//        Serial.println(az); //Serial.print("\t");
//        Serial.print(gx); Serial.print("\t");
//        Serial.print(gy); Serial.print("\t");
//        Serial.println(gz);
//    #endif

//    #ifdef OUTPUT_BINARY_ACCELGYRO      // This is probably what we want to use
//        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));   // accel x-axis
//        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));   // accel y-axis
//        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));   // accel z-axis
//        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));   // gyro x-axis
//        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));   // gyro y-axis
//        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));   // gyro z-axis
//    #endif

}
