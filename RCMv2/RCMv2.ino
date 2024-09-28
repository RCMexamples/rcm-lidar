//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information see this page: https://github.com/RCMgames

/**
uncomment one of the following lines depending on which hardware you have
Remember to also choose the "environment" for your microcontroller in PlatformIO
*/
// #define RCM_HARDWARE_VERSION RCM_ORIGINAL // versions 1, 2, 3, and 3.1 of the original RCM hardware // https://github.com/RCMgames/RCM_hardware_documentation_and_user_guide
// #define RCM_HARDWARE_VERSION RCM_4_V1 // version 1 of the RCM 4 // https://github.com/RCMgames/RCM-Hardware-V4
#define RCM_HARDWARE_VERSION RCM_BYTE_V2 // version 2 of the RCM BYTE // https://github.com/RCMgames/RCM-Hardware-BYTE
// #define RCM_HARDWARE_VERSION RCM_NIBBLE_V1 // version 1 of the RCM Nibble // https://github.com/RCMgames/RCM-Hardware-Nibble
// #define RCM_HARDWARE_VERSION RCM_D1_V1 // version 1 of the RCM D1 // https://github.com/RCMgames/RCM-Hardware-D1

/**
uncomment one of the following lines depending on which communication method you want to use
*/
// #define RCM_COMM_METHOD RCM_COMM_EWD // use the normal communication method for RCM robots
#define RCM_COMM_METHOD RCM_COMM_ROS // use the ROS communication method

#include "rcm.h" //defines pins

// set up motors and anything else you need here
// See this page for how to set up servos and motors for each type of RCM board:
// https://github.com/RCMgames/useful-code/tree/main/boards
// See this page for information about how to set up a robot's drivetrain using the JMotor library
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

JMotorDriverEsp32Servo servo1 = JMotorDriverEsp32Servo(port1);
JMotorDriverEsp32Servo servo2 = JMotorDriverEsp32Servo(port2);
JMotorDriverServoDual servosDriver = JMotorDriverServoDual(servo1, servo2);

int maxServoAngle = 320;
JServoController servos = JServoController(servosDriver, true, INFINITY, INFINITY, INFINITY, 0, 0, 315, 0, 0, 315, 544, 2600);

#include <VL53L0X.h>
VL53L0X sensor;

void Enabled()
{
    // code to run while enabled, put your main code here
}

void Enable()
{
    // turn on outputs
}

void Disable()
{
    // turn off outputs
}

#include "sensor_msgs/msg/laser_scan.h"
declarePub(laserScan, sensor_msgs__msg__LaserScan);
int increment;
int start;
int end;
void initializeLaserScan()
{
    // just to keep things organized
    createPublisher(laserScan, sensor_msgs__msg__LaserScan, "/scan");

    laserScanMsg.header.frame_id.capacity = strlen("base_link") + 1;
    laserScanMsg.header.frame_id.data = (char*)malloc(laserScanMsg.header.frame_id.capacity);
    memcpy(laserScanMsg.header.frame_id.data, "base_link", laserScanMsg.header.frame_id.capacity);
    laserScanMsg.header.frame_id.size = laserScanMsg.header.frame_id.capacity;

    increment = 10;
    start = 0;
    end = maxServoAngle;

    laserScanMsg.angle_increment = radians(increment);
    laserScanMsg.angle_min = radians(start);
    laserScanMsg.angle_max = radians(end);
    laserScanMsg.time_increment = .05;
    laserScanMsg.scan_time = 1;
    laserScanMsg.range_min = .01;
    laserScanMsg.range_max = 1.0;

    laserScanMsg.intensities.capacity = abs(end - start) / increment + 1;
    laserScanMsg.intensities.data = (float*)malloc(laserScanMsg.intensities.capacity * sizeof(float));
    laserScanMsg.intensities.size = laserScanMsg.intensities.capacity;

    laserScanMsg.ranges.capacity = abs(end - start) / increment + 1;
    laserScanMsg.ranges.data = (float*)malloc(laserScanMsg.ranges.capacity * sizeof(float));
    laserScanMsg.ranges.size = laserScanMsg.ranges.capacity;
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    Wire1.begin();
    sensor.setBus(&Wire1);
    sensor.setTimeout(100);
    sensor.setAddress(0x29);
    sensor.setMeasurementTimingBudget(20000);
    sensor.init();
    sensor.startContinuous();

    servos.enable();
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you shouldn't also call leftMotorController.run())
    Serial.println(sensor.readRangeContinuousMillimeters() / 1000.0);

    int64_t time_ns = rmw_uros_epoch_nanos();
    laserScanMsg.header.stamp.sec = time_ns / 1000000000;
    laserScanMsg.header.stamp.nanosec = time_ns % 1000000000;

    Serial.print(laserScanMsg.header.stamp.sec);
    Serial.print(".");
    Serial.println(laserScanMsg.header.stamp.nanosec);

    servos.setAngleSmoothed(0);
    rosSpin(750);
    int n = 0;
    for (int i = start; i < end; i += increment) {
        servos.setAngleSmoothed(i);
        rosSpin(50);
        laserScanMsg.ranges.data[n] = sensor.readRangeContinuousMillimeters() / 1000.0;
        laserScanMsg.intensities.data[n] = 1;
        n++;
    }
    publish(laserScan);
    rosSpin(1);
}

#if RCM_COMM_METHOD == RCM_COMM_EWD
void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;

    // EWD::mode = EWD::Mode::createAP;
    // EWD::APName = "rcm0";
    // EWD::APPassword = "rcmPassword";
    // EWD::APPort = 25210;
}
#elif RCM_COMM_METHOD == RCM_COMM_ROS ////////////// ignore everything below this line unless you're using ROS mode/////////////////////////////////////////////
void ROSWifiSettings()
{
    // SSID, password, IP, port (on a computer run: sudo docker run -it --rm --net=host microros/micro-ros-agent:iron udp4 --port 8887 )
    set_microros_wifi_transports("router", "password", "10.0.0.171", 8887); // doesn't complete until it connects to the wifi network
    nodeName = "rcm_robot";
    // numSubscribers = 10; //change max number of subscribers
}

#include <example_interfaces/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
// and lots of other message types are available (see file available_ros2_types)
// #include <geometry_msgs/msg/twist.h>

// declare publishers
declarePub(battery, std_msgs__msg__Float32);

// // declare subscribers and write callback functions
// declareSubAndCallback(cmd_vel, geometry_msgs__msg__Twist);
// velCmd.x = cmd_velMsg->linear.x;
// velCmd.y = cmd_velMsg->linear.y;
// velCmd.theta = cmd_velMsg->angular.z;
// } // end of callback

void ROSbegin()
{
    // create publishers
    createPublisher(battery, std_msgs__msg__Float32, "/rcm/battery");
    batteryMsg.data = 0;

    initializeLaserScan();

    // add subscribers
    // addSub(cmd_vel, geometry_msgs__msg__Twist, "/cmd_vel");

    RCSOFTCHECK(rmw_uros_sync_session(100)); // allows rmw_uros_epoch_nanos() (useful for timestamping headers)
}

void ROSrun()
{
    rosSpin(1);
    // you can add more publishers here
    batteryMsg.data = voltageComp.getSupplyVoltage();
    publish(battery);
}
#endif

#include "rcmutil.h"
