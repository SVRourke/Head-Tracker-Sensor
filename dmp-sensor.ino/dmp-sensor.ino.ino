#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

#define INTERRUPT_PIN 7 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorFloat gravity;    // [x, y, z]            gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(mpu.testConnection() ? F("calibration ready") : F("error"));

//    // wait for ready
//    Serial.println(F());
    while (Serial.available() && Serial.read());
//    // wait for data
    while (!Serial.available());
//    // empty buffer again
    while (Serial.available() && Serial.read());

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-2159);
    mpu.setYAccelOffset(1115);
    mpu.setZAccelOffset(2970);

    mpu.setXGyroOffset(-21);
    mpu.setYGyroOffset(35);
    mpu.setZGyroOffset(12);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        Serial.println("CALIBRATION FINISHED");

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
    }
    else
    {
        // ERROR!
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // ypr should be called yrp due to sensor orientation
        // originally ypr[x] * 180 / M_PI
        String outPut = "";
        outPut.concat(String(ypr[0] * 10000));
        outPut.concat("|");
        outPut.concat(String(ypr[1] * 10000));
        outPut.concat("|");
        outPut.concat(String(ypr[2] * 10000));
        Serial.println(outPut);
        
//        Serial.print(String(ypr[0]));
//        Serial.print("\t");
//        Serial.print(String(ypr[1]));
//        Serial.print("\t");
//        Serial.println(String(ypr[2]));

    }
}
