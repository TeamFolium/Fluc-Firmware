#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//#define OUTPUT_READABLE_YAWPITCHROLL // yaw/pitch/roll angles calculated from the quaternions coming from the FIFO.
#define INTERRUPT_PIN 2 // Interrupt pin

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//INTERRUPT DETECTION FUNCTION:
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    devStatus = mpu.dmpInitialize(); // Initialize the device.

    // CALIBRATION ROUTINE, USE THE CALIBRATION SKETCH AND WRITE THE VALUES DOWN.
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    // TODO: ACCELERATION CALIBRATION

    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true); // The DMP is ready, enable it.
        // Enable interrupt detection:
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true; // DMP is ready to use, notify it with a flag!
        // Get expected DMP packet size for later comparison:
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else // Error! 1 = Initial memory load failed. 2 = DMP configuration updates failed.
    {
        // ERROR REPORTING (SERIAL MONITOR):
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {
    // if the initialization failed, don't do anything:
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // TODO: other program behavior
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Display yaw/pitch/roll in degrees:
    mpu.dmpGetQuaternion(&q, fifoBuffer); // Gets current Quaternion from FIFO.
    mpu.dmpGetGravity(&gravity, &q); // Calculates gravity.
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Gets yaw/pitch/roll
}
