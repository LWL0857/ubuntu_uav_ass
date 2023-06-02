/***********************************************************************
Copyright (c) 2022

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <chrono>
#include <memory>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include <thread>
#include <algorithm>
#include <csignal>
#include <stdlib.h>
#include <serial/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "uav_msgs/msg/uav_status.hpp"
#include "uav_msgs/srv/uav_led.hpp"
#include "uav_msgs/srv/uav_buzzer.hpp"
#include "uav_msgs/srv/uav_pid.hpp"
#include "uav_msgs/srv/uav_mocap.hpp"
#include "uav_msgs/srv/uav_uwb.hpp"
//mocap数据类型
#include "geometry_msgs/PoseStamped.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#define UAV_WHEEL_TRACK  (0.11)
//数据
//mocap uwb 数据都是float,4byte
union DataFloat
{
    float d;
    unsigned char data[4];
};
// uav protocol data format
typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t length;
    //7*4=28
    uint8_t data[28];
    uint8_t check;
    uint8_t tail;
} DataFrame;

typedef struct {
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angular_x;
    float angular_y;
    float angular_z;
    float roll;
    float pitch;
    float yaw;
} DataImu;

typedef struct {
    float battery_voltage;
    bool buzzer_on;
    bool led_on;
} RobotStatus;

enum {
    FRAME_ID_MOTION       = 0x01,
    FRAME_ID_VELOCITY     = 0x02,
    FRAME_ID_ACCELERATION = 0x03,
    FRAME_ID_ANGULAR      = 0x04,
    FRAME_ID_EULER        = 0x05,
    FRAME_ID_SENSOR       = 0x06,
    FRAME_ID_HMI          = 0x07,
    FRAME_ID_UWB          = 0x08,
    FRAME_ID_MOCAP        = 0x09,

    FRAME_ID_STATUS       = 0x01,
    FRAME_ID_IMU          = 0x02,
    FRAME_ID_MAG          = 0x03,
    FRAME_ID_UWB          = 0x04,   
};

class UavBase : public rclcpp::Node
{
public:
    UavBase(std::string nodeName);
    ~UavBase();

    void driver_loop();
    
private:
    void readRawData();
    bool checkDataFrame(DataFrame &frame);
    void processDataFrame(DataFrame &frame);

    void processVelocityData(DataFrame &frame);
    void processAngularData(DataFrame &frame);
    void processAccelerationData(DataFrame &frame);
    void processEulerData(DataFrame &frame);
    void processSensorData(DataFrame &frame);
    void processUwbData(DataFrame &frame)

    double imu_conversion(uint8_t data_high, uint8_t data_low);
    bool   imu_calibration();
    double degToRad(double deg);

    void odom_publisher(float vx, float vth);
    void imu_publisher();

    bool buzzer_control(bool on);
    bool led_control(bool on);

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void buzzer_callback(const std::shared_ptr<Uav_msgs::srv::UavBuzzer::Request>  request,
                               std::shared_ptr<Uav_msgs::srv::UavBuzzer::Response> response);
    void led_callback(const std::shared_ptr<Uav_msgs::srv::UavLed::Request>  request,
                            std::shared_ptr<Uav_msgs::srv::UavLed::Response> response);
    void left_pid_callback(const std::shared_ptr<Uav_msgs::srv::UavPID::Request>  request,
                            std::shared_ptr<Uav_msgs::srv::UavPID::Response> response);
    void right_pid_callback(const std::shared_ptr<Uav_msgs::srv::UavPID::Request>  request,
                            std::shared_ptr<Uav_msgs::srv::UavPID::Response> response);

    void timer_100ms_callback();

private:
    serial::Serial serial_;
    rclcpp::Time current_time_;
    float ahrsEular_x_=0.0,ahrsEular_y_=0.0,ahrsEular_z_=0.0,height_=0;
    uint8_t mode_=0;
    uint8_t lock_=0;

    float acc_x_=0.0,acc_y_=0.0,acc_z_=0.0;
    float gyro_x_=0.0,gyro_y_=0.0,gyro_z_=0.0;
    float magraw_x_=0.0,magraw_y_=0.0,magraw_z_=0.0;

    float odom_x_=0.0, odom_y_=0.0, odom_th_=0.0;

    float correct_factor_vx_ = 1.0;
    float correct_factor_vth_ = 1.0;    
    
    std::shared_ptr<std::thread> read_data_thread_;
    DataImu imu_data_;
    UavUwb uwb_data_;
    RobotStatus robot_status_;

    rclcpp::TimerBase::SharedPtr timer_100ms_;
    bool auto_stop_on_ = true;
    bool use_imu_ = false;
    bool pub_odom_ = false;
    bool use_mocap_=false;
    bool use_uwb_=false;
    unsigned int auto_stop_count_ = 0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<originbot_msgs::msg::UavStatus>::SharedPtr status_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
   
    rclcpp::Service<originbot_msgs::srv::OriginbotBuzzer>::SharedPtr buzzer_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotLed>::SharedPtr led_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotPID>::SharedPtr left_pid_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotPID>::SharedPtr right_pid_service_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};