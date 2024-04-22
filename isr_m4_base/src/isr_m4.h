#ifndef ISR_M4_H
#define ISR_M4_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <isr_m4_base/RobotStatusStamped.h>
#include <isr_m4_base/RobotCommand.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#define WHEEL_RADIUS_M 0.155  // Unit:m
#define WHEEL_BASE_M 0.531    // Unit:m
#define WHEEL_WIDTH_M 0.102   // Unit:m
#define ENCODER_PPR 6400.0    // Unit: pulse/rev
#define GEAR_RATIO 31.778     // Gearhead reduction ratio: 26 (26:1), Spurgear reduction ratio: 1.22 (44:36)
#define MPS2RPM 61.608        // same as (60 /(2 * M_PI * WHEEL_RADIUS_M))
#define MAX_RPM 4650.0

using namespace std::chrono_literals;

namespace isr_robot
{
class M4
{
public:
  const M4& operator=(const M4&) = delete;
  M4(const M4&) = delete;

  static std::shared_ptr<M4> create(ros::NodeHandle& nh)
  {
    auto isr_m4_shared_ptr = std::shared_ptr<M4>(new M4(nh));
    isr_m4_shared_ptr->weak_self_ = isr_m4_shared_ptr;
    return isr_m4_shared_ptr;
  }

  auto createQuaternionMsgFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  bool ConnectRobot(const std::string& port, const int baudrate);
  void DisconnectRobot(void);

  bool Initialize(void);

  bool EnableMotors(void);
  bool DisableMotors(void);
  bool StopMotors(void);
  bool ResumeMotors(void);

  bool SetVelocity(double leftWheelVel_MPS, double rightWheelVel_MPS);  // m/s, m/s
  bool SetVelocityVW(double linearVel_MPS, double angularVel_RPS);      // m/s, rad/s
  bool ReadVelocity_ADCApprox();

  bool ReadEncoder(void);
  bool ResetRobotPos(void);

  bool ReadRobotStatus(uint8_t& motorEnableStatus, uint8_t& motorStopStatus, uint8_t& emergencyButtonPressed);

public:
  ros::Time prev_encoder_time_;  // ms
  ros::Time cur_encoder_time_;   // ms

  long left_encoder_;   // A encoder value of left wheel (Unit: pulse)
  long right_encoder_;  // A encoder value of right wheel (Unit: pulse)

  bool left_wheel_direction_reading_;
  bool right_wheel_direction_reading_;
  double left_wheel_vel_reading_mps_;
  double right_wheel_vel_reading_mps_;

  struct Position
  {
    double x;
    double y;
    double theta;
  } position_;  // Robot pose calculated by dead-reckoning (Unit: m, m, rad)

  struct Velocity
  {
    double v;
    double w;
  } velocity_;  // Robot velocity calculated by dead-reckoning (Unit: m, m, rad)

private:
  std::weak_ptr<M4> weak_self_;
  ros::NodeHandle nh_;
  M4(ros::NodeHandle& nh) : nh_(nh), serial_(io)
  {
    // Initialize cmd_vel msg subscription
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &M4::cmd_vel_callback, this);
    cmd_vel_msg_.linear.x;
    cmd_vel_msg_.angular.z;

    // Initialize odometry msg publisher
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

    // Initialize robot status msg publisher
    robot_status_pub_ = nh_.advertise<isr_m4_base::RobotStatusStamped>("robot_status", 10);
    robot_status_msg_.header.stamp = ros::Time::now();
    robot_status_msg_old_.header.stamp = ros::Time::now();

    // Initialize robot command service server
    robot_cmd_srv_ = nh_.advertiseService("robot_cmd", &M4::robot_cmd_callback, this);
  }

  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr msg)
  {
    cmd_vel_msg_ = *msg;
    ROS_INFO_STREAM("Received cmd_vel << "
                    << "v: " << this->cmd_vel_msg_.linear.x << " / w: " << this->cmd_vel_msg_.angular.z);
  }

  bool robot_cmd_callback(isr_m4_base::RobotCommand::Request& request, isr_m4_base::RobotCommand::Response& response)
  {
    while (ros::ok())
    {
      if (this->serial_io_mut_.try_lock())
      {
        response.result = isr_m4_base::RobotCommand::Response::RESULT_FAIL;
        switch (request.command)
        {
          case isr_m4_base::RobotCommand::Request::COMMAND_INITIALIZE:
            ROS_INFO("Initialize motor");
            if (Initialize())
              response.result = isr_m4_base::RobotCommand::Response::RESULT_SUCCESS;
            else
              ROS_ERROR("Initialize motor failed");
            break;
          case isr_m4_base::RobotCommand::Request::COMMAND_ENABLE_MOTOR:
            if (request.val == 0)
            {
              ROS_INFO("Enable motor");
              if (EnableMotors())
                response.result = isr_m4_base::RobotCommand::Response::RESULT_SUCCESS;
              else
                ROS_ERROR("Enable motor failed");
            }
            else
            {
              ROS_INFO("Disable motor");
              if (DisableMotors())
                response.result = isr_m4_base::RobotCommand::Response::RESULT_SUCCESS;
              else
                ROS_ERROR("Disable motor failed");
            }
            break;
          case isr_m4_base::RobotCommand::Request::COMMAND_STOP_MOTOR:
            if (request.val == 0)
            {
              ROS_INFO("Stop motor");
              if (StopMotors())
                response.result = isr_m4_base::RobotCommand::Response::RESULT_SUCCESS;
              else
                ROS_ERROR("Stop motor failed");
            }
            else
            {
              ROS_INFO("Resume motor");
              if (ResumeMotors())
                response.result = isr_m4_base::RobotCommand::Response::RESULT_SUCCESS;
              else
                ROS_ERROR("Resume motor failed");
            }
            break;
          default:
            break;
        }

        // ROS_INFO("received request.command: %d", request.command);
        // ROS_INFO("received request.val: %d", request.val);
        // ROS_INFO("sent response.result: %d", response.result);

        this->serial_io_mut_.unlock();
        break;
      }
    }

    if (response.result == isr_m4_base::RobotCommand::Response::RESULT_FAIL)
      return false;

    return true;
  }

  // serial io
  boost::asio::io_service io;
  boost::asio::serial_port serial_;  /// @brief Actual serial port object for reading/writing to robot

  // serial io thread
  ros::WallTimer timer_;
  std::mutex serial_io_mut_;

  // cmd_vel subscription
  ros::Subscriber cmd_vel_sub_;
  geometry_msgs::Twist cmd_vel_msg_;

  // odometry publisher
  ros::Publisher odom_pub_;
  bool publish_tf_{ nh_.param<bool>("publish_tf", true) };

  // robot status publisher
  ros::Publisher robot_status_pub_;
  isr_m4_base::RobotStatusStamped robot_status_msg_;
  isr_m4_base::RobotStatusStamped robot_status_msg_old_;

  // robot command service server
  ros::ServiceServer robot_cmd_srv_;

  /**
   * @brief DeadReckoning
   * Do not need to use this func. 'SetVel' func calls this function.
   */
  void DeadReckoning(long dl, long dr);  // Do not need to use this func. 'SetVel' func calls this function.

  bool SendData(uint8_t command, uint8_t numparam, uint8_t* params);
  std::vector<uint8_t> ReceiveData(uint8_t& command);

  std::vector<uint8_t> Word2Bytes(int16_t dat);
  std::vector<uint8_t> Long2Bytes(int32_t dat);
  int16_t Bytes2Word(uint8_t* data);
  int32_t Bytes2Long(uint8_t* data);
};

}  // namespace isr_robot

#endif  // ISR_M4_H