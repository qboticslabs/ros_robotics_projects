// These are the Arduino headers to read IMU values using I2C protocoal. 
//It also include Arduino - MPU 9150 headers for performing special functions.

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Creating a MPU6050 handle which can be used for MPU 9250
MPU6050 mpu;

// These are the ROS headers for getting ROS Client API's.
#include <ros.h>

//Header for Vector3 ROS message
#include <geometry_msgs/Vector3.h>

//Header for TF broadcast
#include <tf/transform_broadcaster.h>

//These are the object handlers of TF message and broadcaster 
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;


//Creating handlers of Node, IMU message, quaternion and ROS publisher.
ros::NodeHandle nh;

//Creating orientation message header 
geometry_msgs::Vector3 orient;

//Creating ROS publisher object for IMU orientation
ros::Publisher imu_pub("imu_data", &orient);

//The frame_id helps to visulize the Transform data of IMU w.r.t this link
char frameid[] = "/base_link";
char child[] = "/imu_frame";


//Defining an LED pin to show the status of IMU data read
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
// This is the setup function of Arduino
// This will initialize I2C communication, ROS node handle, TF publisher, ROS publisher and DMP of IMU 
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    nh.initNode();
    broadcaster.init(nh);

    nh.advertise(imu_pub);

    mpu.initialize();

    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      
          ;
          }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    
    
    
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
          ;
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();


    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x01) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            //Assigning YAW,PITCH,ROLL to vector message and publishing the values
            
            orient.x = ypr[0] * 180/M_PI;
            orient.y = ypr[1] * 180/M_PI;
            orient.z = ypr[2] * 180/M_PI;
            imu_pub.publish(&orient);

            //Assigning values to TF header and publish the transform
            t.header.frame_id = frameid;
            t.child_frame_id = child;
            t.transform.translation.x = 1.0; 
            t.transform.rotation.x = q.x;
            t.transform.rotation.y = q.y; 
            t.transform.rotation.z = q.z; 
            t.transform.rotation.w = q.w;  
            t.header.stamp = nh.now();
            broadcaster.sendTransform(t);



            nh.spinOnce();
   

            delay(200);
          
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
            delay(200);
    }
}
