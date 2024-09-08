// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * tag_can_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 */

#include <iostream>

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "sensor_msgs/Imu.h"
#include <diagnostic_updater/diagnostic_updater.h>

#define DIAG_OK diagnostic_msgs::DiagnosticStatus::OK
#define DIAG_WARN diagnostic_msgs::DiagnosticStatus::WARN
#define DIAG_ERROR diagnostic_msgs::DiagnosticStatus::ERROR

const uint8_t BIT_FLAG_0 = (1 << 0);  // 0000 0001
const uint8_t BIT_FLAG_1 = (1 << 1);  // 0000 0010
const uint8_t BIT_FLAG_2 = (1 << 2);  // 0000 0100
const uint8_t BIT_FLAG_3 = (1 << 3);  // 0000 1000
const uint8_t BIT_FLAG_4 = (1 << 4);  // 0001 0000
const uint8_t BIT_FLAG_5 = (1 << 5);  // 0010 0000
const uint8_t BIT_FLAG_6 = (1 << 6);  // 0100 0000
const uint8_t BIT_FLAG_7 = (1 << 7);  // 1000 0000

static unsigned int counter;
static unsigned int prev_cnt;
static int16_t raw_data;
static int32_t raw_data2;
static uint16_t dbg_imu_status; // debug
static uint imu_status[16];
static bool use_fog;
static bool ready = false;

static diagnostic_updater::Updater* p_updater;

static sensor_msgs::Imu imu_msg;
static ros::Publisher pub;

void receive_can_callback(const can_msgs::Frame::ConstPtr& msg){

  if(msg->id == 0x319)
  {
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = msg->header.stamp;

    if (use_fog) 
    {
      raw_data = msg->data[1] + (msg->data[0] << 8);
      imu_msg.angular_velocity.x =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
      raw_data = msg->data[3] + (msg->data[2] << 8);
      imu_msg.angular_velocity.y =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
      raw_data2 = (msg->data[7] + (msg->data[6] << 8)) + ((msg->data[5] << 16) + (msg->data[4] << 24));
      imu_msg.angular_velocity.z =
          raw_data2 * (200 / pow(2, 31)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    }
    else 
    {
      counter = msg->data[1] + (msg->data[0] << 8);
      raw_data = msg->data[3] + (msg->data[2] << 8);
      imu_msg.angular_velocity.x =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
      raw_data = msg->data[5] + (msg->data[4] << 8);
      imu_msg.angular_velocity.y =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
      raw_data = msg->data[7] + (msg->data[6] << 8);
      imu_msg.angular_velocity.z =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    }
    
  }
  else if(msg->id == 0x31A)
  {
    dbg_imu_status = msg->data[1] + (msg->data[0] << 8); // debug

    imu_status[0] =  msg->data[1] & BIT_FLAG_0;
    imu_status[1] = (msg->data[1] & BIT_FLAG_1) >> 1;
    imu_status[2] = (msg->data[1] & BIT_FLAG_2) >> 2;
    imu_status[3] = (msg->data[1] & BIT_FLAG_3) >> 3;
    imu_status[4] = (msg->data[1] & BIT_FLAG_4) >> 4;
    imu_status[5] = (msg->data[1] & BIT_FLAG_5) >> 5;
    imu_status[6] = (msg->data[1] & BIT_FLAG_6) >> 6;
    imu_status[7] = (msg->data[1] & BIT_FLAG_7) >> 7;

    imu_status[8]  =  msg->data[0] & BIT_FLAG_0;
    imu_status[9]  = (msg->data[0] & BIT_FLAG_1) >> 1;
    imu_status[10] = (msg->data[0] & BIT_FLAG_2) >> 2;
    imu_status[11] = (msg->data[0] & BIT_FLAG_3) >> 3;
    imu_status[12] = (msg->data[0] & BIT_FLAG_4) >> 4;
    imu_status[13] = (msg->data[0] & BIT_FLAG_5) >> 5;
    imu_status[14] = (msg->data[0] & BIT_FLAG_6) >> 6;
    imu_status[15] = (msg->data[0] & BIT_FLAG_7) >> 7;

    raw_data = msg->data[3] + (msg->data[2] << 8);
    imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
    raw_data = msg->data[5] + (msg->data[4] << 8);
    imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
    raw_data = msg->data[7] + (msg->data[6] << 8);
    imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    pub.publish(imu_msg);

    ready = true;
    std::cout << dbg_imu_status << std::endl;
  }
}

static void check_connection(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (counter == prev_cnt) {
    stat.summary(DIAG_ERROR, "New message is not received!");
    ready = false;
  } else {
    stat.summary(DIAG_OK, "OK");
  }

  prev_cnt = counter;
  stat.add("counter", counter);
}

static void check_calc_mode(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }

  uint8_t calc_mode = imu_status[0] + imu_status[1]*2 + imu_status[2]*4 + imu_status[3]*8;
  switch (calc_mode)
  {
  case 1:
    stat.summary(DIAG_WARN, "Initializing");
    break;
  case 3:
    stat.summary(DIAG_WARN, "Offset cancelling");
    break;
  case 4:
    stat.summary(DIAG_OK, "Leveling calcuration");
    break;
  default:
    stat.summary(DIAG_WARN, "Undefined status!");
    break;
  }

  stat.add("imu_status[0]", imu_status[0]);
  stat.add("imu_status[1]", imu_status[1]);
  stat.add("imu_status[2]", imu_status[2]);
  stat.add("imu_status[3]", imu_status[3]);
}

static void check_vel_available(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }
  
  if (imu_status[4]) {
    stat.summary(DIAG_OK, "Enabled");
  } else {
    stat.summary(DIAG_WARN, "Disabled");
  }

  stat.add("imu_status[4]", imu_status[4]);
}

static void check_heading_fixed(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }

  if (imu_status[5]) {
    stat.summary(DIAG_WARN, "Heading fixed!");
  } else {
    stat.summary(DIAG_OK, "OK");
  }

  stat.add("imu_status[5]", imu_status[5]);
}

static void check_angler_vel_sts_err(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }

  if (imu_status[8]) {
    stat.summary(DIAG_ERROR, "Anguler velocity status error!");
  } else {
    stat.summary(DIAG_OK, "OK");
  }

  stat.add("imu_status[8]", imu_status[8]);
}

static void check_angler_vel_bias_err(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }

  if (imu_status[10]) {
    stat.summary(DIAG_ERROR, "Anguler velocity bias error!");
  } else {
    stat.summary(DIAG_OK, "OK");
  }

  stat.add("imu_status[10]", imu_status[10]);
}

static void check_acc_bias_err(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }

  if (imu_status[11]) {
    stat.summary(DIAG_OK, "Acceleration bias error!");
  } else {
    stat.summary(DIAG_OK, "OK");
  }

  stat.add("imu_status[11]", imu_status[11]);
}

static void check_internal_err(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }

  if (imu_status[12]) {
    stat.summary(DIAG_ERROR, "Internal error!");
  } else {
    stat.summary(DIAG_OK, "OK");
  }

  stat.add("imu_status[12]", imu_status[12]);
}

static void check_imu_err(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  if (!ready) {
    stat.summary(DIAG_OK, "Unavailable");
    return;
  }

  if (imu_status[15]) {
    stat.summary(DIAG_ERROR, "Something error occurred!");
  } else {
    stat.summary(DIAG_OK, "OK");
  }

  stat.add("imu_status[0] ", imu_status[0]);
  stat.add("imu_status[1] ", imu_status[1]);
  stat.add("imu_status[2] ", imu_status[2]);
  stat.add("imu_status[3] ", imu_status[3]);
  stat.add("imu_status[4] ", imu_status[4]);
  stat.add("imu_status[5] ", imu_status[5]);
  stat.add("imu_status[6] ", imu_status[6]); // not used
  stat.add("imu_status[7] ", imu_status[7]); // not used
  stat.add("imu_status[8] ", imu_status[8]);
  stat.add("imu_status[9] ", imu_status[9]); // not used
  stat.add("imu_status[10]", imu_status[10]);
  stat.add("imu_status[11]", imu_status[11]);
  stat.add("imu_status[12]", imu_status[12]);
  stat.add("imu_status[13]", imu_status[13]); // not used
  stat.add("imu_status[14]", imu_status[14]); // not used
  stat.add("imu_status[15]", imu_status[15]);
  stat.add("dbg_imu_status", dbg_imu_status); // debug
}

void diagnostic_timer_callback(const ros::TimerEvent& event)
{
  p_updater->force_update();
}

int main(int argc, char **argv){

  ros::init(argc, argv, "tag_can_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Timer diagnostics_timer = nh.createTimer(ros::Duration(1.0), diagnostic_timer_callback);

  pnh.param<bool>("use_fog", use_fog, false);

  diagnostic_updater::Updater updater;
  p_updater = &updater;
  updater.setHardwareID("tamagawa");
  updater.add("connection", check_connection);
  updater.add("calc_mode", check_calc_mode);                        // bit 0-3 角速度バイアスエラー
  updater.add("velocity_available", check_vel_available);           // bit 4   速度入力有無
  updater.add("heading_fixed", check_heading_fixed);                // bit 5   方位角固定
  updater.add("angler_vel_status_err", check_angler_vel_sts_err);   // bit 8   角速度ステータスエラー
  updater.add("angler_vel_bias_err", check_angler_vel_bias_err);    // bit 10  角速度バイアスエラー
  updater.add("acc_bias_err", check_acc_bias_err);                  // bit 11  加速度バイアスエラー
  updater.add("internal_err", check_internal_err);                  // bit 12  その他エラー
  updater.add("imu_error", check_imu_err);                          // bit 15  エラー有無
  
  ros::Subscriber sub = nh.subscribe("can_tx", 100, receive_can_callback);
  pub = nh.advertise<sensor_msgs::Imu>("data_raw", 100);
  ros::spin();

  return 0;
}
