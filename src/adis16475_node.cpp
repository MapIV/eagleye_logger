// Copyright (c) 2020, Map IV, Inc.
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

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "can_msgs/Frame.h"
#include "eagleye_logger_driver/pps.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include <termios.h>
#include <sys/socket.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "eagleye_logger_driver/gnss_server.h"

#define GNSS_PACKECT_LEN 2
#define CMD_NUM_MAX (79)
#define BUFF_SIZE 1024

uint32_t imu_time;
// Gyro sensor(x, y, z)
double imu_gyro[3];
// Acceleration sensor(x, y, z)
double imu_accl[3];
// Temperature sensor
double imu_temp;

unsigned char recv_buff[BUFF_SIZE] = {0};
unsigned char buff[BUFF_SIZE] = {0};
GnssServer server;


void parseIMUdata( unsigned char* imu_data ){

  int16_t gyro_out[3], gyro_low[3], accl_out[3], accl_low[3], temp_out, time_stamp;
  gyro_low[0] = (int16_t)(*(imu_data+5) << 8) + (int16_t)(*(imu_data+6));
  gyro_out[0] = (int16_t)(*(imu_data+7) << 8) + (int16_t)(*(imu_data+8));
  gyro_low[1] = (int16_t)(*(imu_data+9) << 8) + (int16_t)(*(imu_data+10));
  gyro_out[1] = (int16_t)(*(imu_data+11) << 8) + (int16_t)(*(imu_data+12));
  gyro_low[2] = (int16_t)(*(imu_data+13) << 8) + (int16_t)(*(imu_data+14));
  gyro_out[2] = (int16_t)(*(imu_data+15) << 8) + (int16_t)(*(imu_data+16));

  accl_low[0] = (int16_t)(*(imu_data+17) << 8) + (int16_t)(*(imu_data+18));
  accl_out[0] = (int16_t)(*(imu_data+19) << 8) + (int16_t)(*(imu_data+20));
  accl_low[1] = (int16_t)(*(imu_data+21) << 8) + (int16_t)(*(imu_data+22));
  accl_out[1] = (int16_t)(*(imu_data+23) << 8) + (int16_t)(*(imu_data+24));
  accl_low[2] = (int16_t)(*(imu_data+25) << 8) + (int16_t)(*(imu_data+26));
  accl_out[2] = (int16_t)(*(imu_data+27) << 8) + (int16_t)(*(imu_data+28));

  temp_out = (int16_t)(*(imu_data+29) << 8) + (int16_t)(*(imu_data+30));
  time_stamp = (int16_t)(*(imu_data+31) << 8) + (int16_t)(*(imu_data+32));

  // temperature convert
  imu_temp = temp_out * 0.1;

  // 32bit convert
  for (int i=0; i < 3; i++)
  {
    /* adis16475 */
    imu_gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 2621440.0;
    imu_accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 262144000.0;
  }
}

class logger
{
public:

  //! File descripter for USB-ISS
  int fd_;
  //! Saved terminal config
  struct termios defaults_;

  logger()
  {
    imu_gyro[0] = 0.0;
    imu_gyro[1] = 0.0;
    imu_gyro[2] = 0.0;

    imu_accl[0] = 0.0;
    imu_accl[1] = 0.0;
    imu_accl[2] = 0.0;
  }

  ~logger()
  {

  }

  /**
   * @brief Open device
   * @param device Device file name (/dev/ttyACM*)
   * @retval 0 Success
   * @retval -1 Failure
   */
  int open_serial(const char *device_name){
    fd_=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if( -1 == fd_ ){
      perror("open");
      return fd_;
    }

    fcntl(fd_, F_SETFL,0);
    //load configuration
    struct termios conf_tio;
    tcgetattr(fd_,&conf_tio);
    //set baudrate
    speed_t BAUDRATE = B230400;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    //non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    //non blocking
    conf_tio.c_cc[VMIN]=0;
    conf_tio.c_cc[VTIME]=0;
    //store configuration

    cfmakeraw(&conf_tio);
    tcsetattr(fd_,TCSANOW,&conf_tio);

    struct serial_struct serial_setting;
    ioctl(fd_, TIOCGSERIAL, &serial_setting);
    serial_setting.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCGSERIAL, &serial_setting);

    return fd_;
  }

  /**
   * @brief Close device
   */
  void closePort()
  {
    if (tcsetattr(fd_, TCSANOW, &defaults_) < 0)
    {
      perror("closePort");
    }
    close(fd_);
  }
};
/* ------------------------------------------------------- */

class LoggerNode
{
public:
  logger target;
  ros::NodeHandle node_handle_;
  ros::Publisher sensor_data_pub_;
  ros::Publisher vehicle_data_pub_;
  ros::Publisher pps_data_pub_;

  std::string device_;
  bool debug_;

  std::string topic_name_imu_;
  std::string topic_name_vehicle_;
  std::string topic_name_pps_;

  explicit LoggerNode(ros::NodeHandle nh)
    : node_handle_(nh)
  {
    int tcp_port;

    // Read parameters
    node_handle_.param("/eagleye_logger/device_", device_, std::string("/dev/ttyACM0"));
    node_handle_.param("/eagleye_logger/debug_", debug_, false);

    node_handle_.param("/eagleye_logger/topic_name_imu", topic_name_imu_, std::string("/imu/data_raw"));
    node_handle_.param("/eagleye_logger/topic_name_vehicle", topic_name_vehicle_, std::string("/can/vehicle"));
    node_handle_.param("/eagleye_logger/topic_name_pps", topic_name_pps_, std::string("/pps"));

    node_handle_.param("/eagleye_logger/gnss_server_port", tcp_port, 61130);

    ROS_INFO("device: %s", device_.c_str());

    // Data publisher
    sensor_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>(topic_name_imu_, 100);
    vehicle_data_pub_ = node_handle_.advertise<can_msgs::Frame>(topic_name_vehicle_, 100);
    pps_data_pub_ = node_handle_.advertise<eagleye_logger_driver::pps>(topic_name_pps_, 100);

    //GNSS server
    server.open(tcp_port);
  }

  ~LoggerNode()
  {
    target.closePort();
  }

  /**
   * @brief Open IMU device file
   */
  bool open(void)
  {
    int fd1 = 0;
    // Open device file
    fd1 = target.open_serial(device_.c_str());
    if (fd1 < 0)
    {
      ROS_ERROR("Failed to open device %s", device_.c_str());
      return fd1;
    }

    return fd1;
  }
  void close()
  {
    target.closePort();
  }

  void publish_imu_data()
  {
    sensor_msgs::Imu data;
    data.header.frame_id = "imu";
    data.header.stamp = ros::Time::now();

    data.linear_acceleration.x = imu_accl[0];
    data.linear_acceleration.y = imu_accl[1];
    data.linear_acceleration.z = imu_accl[2];

    data.angular_velocity.x = imu_gyro[0];
    data.angular_velocity.y = imu_gyro[1];
    data.angular_velocity.z = imu_gyro[2];

    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;

    sensor_data_pub_.publish(data);
  }

  void parseCANdata(unsigned char* can_data, uint8_t len){
    can_msgs::Frame data;

    data.header.frame_id = "vehicle";
    data.header.stamp = ros::Time::now();

    uint16_t can_id = ( (*(can_data+6) <<8) | *(can_data+7) );
    data.id = (uint32_t)can_id;
    data.is_rtr = false;
    data.is_extended = false;
    data.is_error = false;
    data.dlc = len;
    memcpy( &data.data, can_data+8, len);

    vehicle_data_pub_.publish(data);
  }


  void parsePPSdata(unsigned char* buf)
  {
    eagleye_logger_driver::pps data;

    data.header.frame_id = "pps";
    data.header.stamp = ros::Time::now();

    data.log_time = (uint64_t)(*(buf+1) << 24);
    data.log_time |= (uint64_t)(*(buf+2) << 16);
    data.log_time |= (uint64_t)(*(buf+3) << 8);
    data.log_time |= (uint64_t)(*(buf+4));

    data.pps_cnt = (uint64_t)(*(buf+5) << 24);
    data.pps_cnt |= (uint64_t)(*(buf+6) << 16);
    data.pps_cnt |= (uint64_t)(*(buf+7) << 8);
    data.pps_cnt |= (uint64_t)(*(buf+8));

    pps_data_pub_.publish(data);
  }

  bool spin()
  {
    uint32_t i,size = 0;

    while (ros::ok())
    {
      ros::spinOnce();

      size = read(target.fd_, buff, sizeof(buff));
      if(size > 0){

        bool isProtcol = false;
        for(int t = 0;t < size; t++){

          if( false == isProtcol ){
            if(((buff[t]&0xFF)==0xF7)&&((buff[t+1]&0xFF)==0xE0)){
              isProtcol = true;
            }
          }else{
            if(buff[t]==0x49){
              parseIMUdata(&buff[t]);
              publish_imu_data();
              t += 32;
            }else if(buff[t]==0x50){
              uint8_t can_len = buff[t+5];
              if( size - t < 8 + can_len ){
                if(debug_){
                  ROS_WARN("CAN PACKET LENGTH OVER len=%d :: PACKET SKIP", can_len );
                }
                t += BUFF_SIZE;
              }else{
                parseCANdata(&buff[t], can_len);
                t += can_len + 7;
              }
            }else if(buff[t]==0x47){
              uint16_t gnss_len = (( (buff[t+1]) <<8) | (buff[t+2]) );
              if( size - t  < gnss_len ){
                if(debug_){
                  ROS_WARN("GNSS PACKET LENGTH OVER len=%d :: PACKET SKIP", gnss_len );
                }
              }else{
                memcpy(&server.sendbuff[server.len], &buff[t+3], (size_t)gnss_len);
                server.len += gnss_len;
              }
              t += gnss_len + GNSS_PACKECT_LEN;

            }else if(buff[t]==0x43){
              parsePPSdata(&buff[t]);
              t += 16;
            }
          }
        }
      }
      /* gnss server task */
      server.isConnect();
    }
  }

};

/* ------------------ */
/* main               */
/* ------------------ */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adis16475_node");
  ros::NodeHandle nh("~");

  LoggerNode node(nh);
  if(node.open() < 0){
    ros::shutdown();
    return 0;
  }

  node.spin();

  return 0;
}
