#include "isr_m3.h"

// Command Definitions
#define COMMAND_INITIALIZE 0

#define COMMAND_MOTOR_ENABLE 1
#define COMMAND_MOTOR_RUN 2
#define COMMAND_MOTOR_STOP 3
#define COMMAND_MOTOR_ACTUAL_SPEED_READ 4
#define COMMAND_MOTOR_ACTUAL_SPEED_READ_RE 5  // for return message

#define COMMAND_ENCODER_READ 11
#define COMMAND_ENCODER_READ_RE 12  // for return message
#define COMMAND_ENCODER_RESET 13

#define COMMAND_STATUS 21
#define COMMAND_STATUS_RE 22  // for return message

namespace isr_robot
{
bool M3::ConnectRobot(const std::string& port, const int baudrate)
{
  try
  {
    serial_.open(port);
  }
  catch (boost::system::system_error& e)
  {
    // color red
    std::cout << "\033[1;31mPort: " << port << "\033[0m" << std::endl;
    std::cout << "\033[1;31mError: " << e.what() << "\033[0m" << std::endl;
    return false;
  }

  try
  {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
  }
  catch (boost::system::system_error& e)
  {
    std::cout << "Unknown baudrate " << baudrate << std::endl;
    return false;
  }

  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  std::this_thread::sleep_for(1s);

  std::cout << "Serial port " << port << " opened at baudrate " << baudrate << std::endl;

  if (!Initialize())
    return false;

  // Declare timer callback for main serial command io
  auto timer_callback = [this](const ros::WallTimerEvent& event) -> void {
    serial_io_mut_.lock();

    // Set linear/angular velocity
    SetVelocityVW(cmd_vel_msg_.linear.x, cmd_vel_msg_.angular.z);
    // ROS_INFO_STREAM("timer cb: set velocity >> "
    // 	<< "v: " << this->cmd_vel_msg_->linear.x
    // 	<< " / w: " << this->cmd_vel_msg_->angular.z);

    // Read encoder
    ReadEncoder();
    // ROS_INFO_STREAM("timer cb: read encoder >> "
    // 	<< "l: " << this->left_encoder_
    // 	<< " / r: " << this->right_encoder_);

    // broadcast odometry transform
    static tf2_ros::TransformBroadcaster odom_tf_broadcaster_;
    geometry_msgs::Quaternion odom_quat = createQuaternionMsgFromYaw(position_.theta);
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = this->cur_encoder_time_;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = position_.x;
    odom_tf.transform.translation.y = position_.y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;
    odom_tf_broadcaster_.sendTransform(odom_tf);

    // publish odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = this->cur_encoder_time_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = position_.x;
    odom.pose.pose.position.y = position_.y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance.fill(0);
    odom.twist.twist.linear.x = velocity_.v;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = velocity_.w;
    odom.twist.covariance.fill(0);
    this->odom_pub_.publish(odom);

    // Read robot status
    static bool robot_status_changed = false;
    ReadRobotStatus(this->robot_status_msg_.motor_enabled, this->robot_status_msg_.motor_stopped,
                    this->robot_status_msg_.estop_pressed);
    // ROS_INFO_STREAM("timer cb: read status >> "
    // 	<< "motor_enabled: "<< this->robot_status_msg_.motor_enabled
    // 	<< " / motor_stopped: " << this->robot_status_msg_.motor_stopped
    // 	<< " / estop_pressed: " << this->robot_status_msg_.estop_pressed);

    if (this->robot_status_msg_old_.motor_enabled != this->robot_status_msg_.motor_enabled)
    {
      ROS_INFO("Motor is %s", (this->robot_status_msg_.motor_enabled ? "ON" : "OFF"));
      robot_status_changed = true;
    }
    if (this->robot_status_msg_old_.motor_stopped != this->robot_status_msg_.motor_stopped)
    {
      ROS_INFO("Motor is %s", (this->robot_status_msg_.motor_stopped ? "Stopped" : "Resumed"));
      robot_status_changed = true;
    }
    if (this->robot_status_msg_old_.estop_pressed != this->robot_status_msg_.estop_pressed)
    {
      ROS_INFO("E-Stop button is %s", (this->robot_status_msg_.estop_pressed ? "Pressed" : "Released"));
      robot_status_changed = true;
    }

    if (robot_status_changed)
    {
      this->robot_status_msg_.header.stamp = this->cur_encoder_time_;
      this->robot_status_pub_.publish(this->robot_status_msg_old_);
      this->robot_status_msg_old_ = this->robot_status_msg_;
      robot_status_changed = false;
    }

    serial_io_mut_.unlock();
  };
  timer_ = this->nh_.createWallTimer(ros::WallDuration(0.01), timer_callback);

  return true;
}

void M3::DisconnectRobot(void)
{
  serial_.close();
}

bool M3::Initialize(void)
{
  if (!SendData(COMMAND_INITIALIZE, 0, NULL))
    return false;

  prev_encoder_time_ = cur_encoder_time_ = ros::Time::now();
  left_encoder_ = right_encoder_ = 0;
  position_.x = position_.y = position_.theta = 0.0;
  velocity_.v = velocity_.w = 0.0;

  return true;
}

bool M3::EnableMotors(void)
{
  uint8_t data[1] = { true };
  if (!SendData(COMMAND_MOTOR_ENABLE, 1, data))
    return false;

  return true;
}
bool M3::DisableMotors(void)
{
  uint8_t data[1] = { false };
  if (!SendData(COMMAND_MOTOR_ENABLE, 1, data))
    return false;

  return true;
}

bool M3::StopMotors(void)
{
  uint8_t data[1] = { true };
  if (!SendData(COMMAND_MOTOR_STOP, 1, data))
    return false;

  return true;
}

bool M3::ResumeMotors(void)
{
  uint8_t data[1] = { false };
  if (!SendData(COMMAND_MOTOR_STOP, 1, data))
    return false;

  return true;
}

bool M3::SetVelocity(double leftWheelVel_MPS, double rightWheelVel_MPS)  // m/s, m/s
{
  // �¿� ���� ������ ����� ������ ȸ���ӵ��� RPM ������ ����Ѵ�.
  int leftMotorlVel_RPM = (int)(leftWheelVel_MPS * MPS2RPM * GEAR_RATIO);
  int rightMotorVel_RPM = (int)(rightWheelVel_MPS * MPS2RPM * GEAR_RATIO);

  // Exceed maximum motor rpm.
  if (leftMotorlVel_RPM > MAX_RPM || rightMotorVel_RPM > MAX_RPM)
  {
    return true;
  }

  uint8_t data[4];
  data[0] = leftMotorlVel_RPM >> 8;
  data[1] = leftMotorlVel_RPM;
  data[2] = rightMotorVel_RPM >> 8;
  data[3] = rightMotorVel_RPM;
  if (!SendData(COMMAND_MOTOR_RUN, 4, data))
    return false;

  //	std::cout << "<Velocity>" << std::endl;
  //	std::cout << "\tLeft wheel : " << leftWheelVel_MPS << "[mps]\t" << leftMotorlVel_RPM << "[rpm]" << std::endl;
  //	std::cout << "\tRight wheel: " << rightWheelVel_MPS << "[mps]\t" << rightMotorVel_RPM << "[rpm]" << std::endl;
  //	std::cout << "<Encoder>" << std::endl;
  //	std::cout << "\tLeft wheel encoder : " << left_encoder_ << "[pulse]" << std::endl;
  //	std::cout << "\tRight wheel encoder: " << right_encoder_ << "[pulse]" << std::endl;
  //	std::cout << "<Position>" << std::endl;
  //	std::cout << "\tX    : " << position_.x << "[m]" << std::endl;
  //	std::cout << "\tY    : " << position_.y << "[m]" << std::endl;
  //	std::cout << "\tTheta: " << position_.theta << "[rad]" << std::endl;

  return true;
}

bool M3::SetVelocityVW(double linearVel_MPS, double angularVel_RPS)  // m/s, rad/s
{
  // left  = MPS2RPM*GEAR_RATIO / WHEEL_RADIUS_M * (linearVel_MPS - WHEEL_BASE_M/2. * angularVel_RPS)
  // right = MPS2RPM*GEAR_RATIO / WHEEL_RADIUS_M * (linearVel_MPS + WHEEL_BASE_M/2. * angularVel_RPS)

  // �¿� ���� ������ ����� ������ ȸ���ӵ��� RPM ������ ����Ѵ�.
  double leftWheelVel_MPS = linearVel_MPS - (WHEEL_BASE_M * angularVel_RPS / 2.);
  int leftMotorVel_RPM = (int)(leftWheelVel_MPS * MPS2RPM * GEAR_RATIO);

  double rightWheelVel_MPS = linearVel_MPS + (WHEEL_BASE_M * angularVel_RPS / 2.);
  int rightMotorVel_RPM = (int)(rightWheelVel_MPS * MPS2RPM * GEAR_RATIO);

  // Exceed maximum motor rpm.
  if (leftMotorVel_RPM > MAX_RPM || rightMotorVel_RPM > MAX_RPM || leftMotorVel_RPM < -MAX_RPM ||
      rightMotorVel_RPM < -MAX_RPM)
    return true;

  uint8_t data[4];
  data[0] = leftMotorVel_RPM >> 8;
  data[1] = leftMotorVel_RPM;
  data[2] = rightMotorVel_RPM >> 8;
  data[3] = rightMotorVel_RPM;
  if (!SendData(COMMAND_MOTOR_RUN, 4, data))
    return false;

  //	std::cout << "<Velocity>" << std::endl;
  //	std::cout << "\tLeft wheel : " << leftWheelVel_MPS << "[mps]\t" << leftMotorVel_RPM << "[rpm]" << std::endl;
  //	std::cout << "\tRight wheel: " << rightWheelVel_MPS << "[mps]\t" << rightMotorVel_RPM << "[rpm]" << std::endl;
  //	std::cout << "<Encoder>" << std::endl;
  //	std::cout << "\tLeft wheel encoder : " << left_encoder_ << "[pulse]" << std::endl;
  //	std::cout << "\tRight wheel encoder: " << right_encoder_ << "[pulse]" << std::endl;
  //	std::cout << "<Position>" << std::endl;
  //	std::cout << "\tX    : " << position_.x << "[m]" << std::endl;
  //	std::cout << "\tY    : " << position_.y << "[m]" << std::endl;
  //	std::cout << "\tTheta: " << position_.theta << "[rad]" << std::endl;

  return true;
}

bool M3::ReadVelocity_ADCApprox(void)
{
  if (!SendData(COMMAND_MOTOR_ACTUAL_SPEED_READ, 0, NULL))
    return false;

  uint8_t command;
  std::vector<uint8_t> data = ReceiveData(command);
  if (data.size() == 0)
    return false;

  if (command == COMMAND_MOTOR_ACTUAL_SPEED_READ_RE && data.size() == 6)
  {
    std::vector<uint8_t>::iterator it;
    uint8_t byteToConvert[2];

    left_wheel_direction_reading_ = data[0];
    right_wheel_direction_reading_ = data[1];

    int position = 0;
    for (it = data.begin() + 2; it < data.end() - 2; it++)
    {
      byteToConvert[position++] = *it;
    }
    int leftMotor_actualRPM_Analog =
        Bytes2Word(byteToConvert);  // Analog input range: 0 ~ 819 (0 ~ 4 VDC) -> RPM Range: 0 ~ MAX_RPM

    position = 0;
    for (it = data.begin() + 4; it < data.end(); it++)
    {
      byteToConvert[position++] = *it;
    }
    int rightMotor_actualRPM_Analog =
        Bytes2Word(byteToConvert);  // Analog input range: 0 ~ 819 (0 ~ 4 VDC) -> RPM Range: 0 ~ MAX_RPM

    double leftMotorlVel_RPM = (double)leftMotor_actualRPM_Analog / 819.2 * MAX_RPM;
    double rightMotorVel_RPM = (double)rightMotor_actualRPM_Analog / 819.2 * MAX_RPM;

    left_wheel_vel_reading_mps_ = leftMotorlVel_RPM / (MPS2RPM * GEAR_RATIO);
    right_wheel_vel_reading_mps_ = rightMotorVel_RPM / (MPS2RPM * GEAR_RATIO);

    if (left_wheel_direction_reading_ == 0)
      left_wheel_vel_reading_mps_ *= -1;
    if (right_wheel_direction_reading_ == 0)
      left_wheel_vel_reading_mps_ *= -1;
  }
  else
  {
    return false;
  }

  return true;
}

bool M3::ReadEncoder(void)
{
  if (!SendData(COMMAND_ENCODER_READ, 0, NULL))
    return false;

  uint8_t command;
  std::vector<uint8_t> data = ReceiveData(command);
  if (data.size() == 0)
    return false;

  if (command == COMMAND_ENCODER_READ_RE && data.size() == 8)
  {
    prev_encoder_time_ = cur_encoder_time_;
    cur_encoder_time_ = ros::Time::now();

    long _leftEncoderPrev = left_encoder_;
    long _rightEncoderPrev = right_encoder_;

    std::vector<uint8_t>::iterator it;
    uint8_t byteToConvert[4];

    int position = 0;
    for (it = data.begin(); it < data.end() - 4; it++)
    {
      byteToConvert[position++] = *it;
    }
    left_encoder_ = Bytes2Long(byteToConvert) / 4;

    position = 0;
    for (it = data.begin() + 4; it < data.end(); it++)
    {
      byteToConvert[position++] = *it;
    }
    right_encoder_ = Bytes2Long(byteToConvert) / 4;

    // Calculates odometry when calls ReadEncoder func
    long dl_pulse = (left_encoder_ - _leftEncoderPrev);
    long dr_pulse = (right_encoder_ - _rightEncoderPrev);

    DeadReckoning(-dl_pulse, -dr_pulse);

    return true;
  }
  else
  {
    return false;
  }

  return false;
}

bool M3::ResetRobotPos(void)
{
  if (!SendData(COMMAND_ENCODER_RESET, 0, NULL))
    return false;

  ReadEncoder();
  position_.x = position_.y = position_.theta = 0.0;

  return true;
}

bool M3::ReadRobotStatus(uint8_t& motorEnableStatus, uint8_t& motorStopStatus, uint8_t& emergencyButtonPressed)
{
  if (!SendData(COMMAND_STATUS, 0, NULL))
    return false;

  uint8_t command;
  std::vector<uint8_t> data = ReceiveData(command);
  if (data.size() == 0)
    return false;

  if (command == COMMAND_STATUS_RE && data.size() == 3)
  {
    motorEnableStatus = data[0];
    motorStopStatus = data[1];
    emergencyButtonPressed = data[2];
  }
  else
  {
    return false;
  }

  return true;
}

// Calculates forward kinematics to get robot's pose(x, y, theta) by changing amount of L/R wheel encoder.
// This is called Dead-reckoning.
// We are going to update position_, velocity_ member (x, y, theta)
void M3::DeadReckoning(long dl_pulse, long dr_pulse)
{
  double scaler = 2.0 * M_PI * WHEEL_RADIUS_M / ENCODER_PPR / GEAR_RATIO;  // 2 pi*r
  double dl = scaler * dl_pulse;
  double dr = scaler * dr_pulse;
  auto diff = (cur_encoder_time_ - prev_encoder_time_).toSec() / (double)1000.0;  // why divided by 1000?

  velocity_.v = (dl + dr) / 2.0;
  velocity_.w = (dr - dl) / WHEEL_BASE_M;

  // alias
  const auto& v = velocity_.v;
  const auto& w = velocity_.w;

  if (-0.001 > w || w > 0.001)
  {
    position_.x += v / w * (sin(position_.theta + w) - sin(position_.theta));
    position_.y -= v / w * (cos(position_.theta + w) - cos(position_.theta));
  }
  else
  {
    position_.x += v * cos(position_.theta);
    position_.y += v * sin(position_.theta);
  }
  position_.theta += w;
}

bool M3::SendData(uint8_t command, uint8_t numparam, uint8_t* params)
{
  uint8_t CRC = numparam + 1;

  boost::array<uint8_t, 128> data_;
  data_[0] = 0x06;
  data_[1] = 0x85;
  data_[2] = numparam + 1;  // command(1) + data.size(n)
  CRC ^= command;
  data_[3] = command;
  for (int i = 0; i < numparam; i++)
  {
    CRC ^= params[i];
    data_[4 + i] = params[i];
  }
  data_[4 + numparam] = CRC;

  //  for (int i = 0; i < 5 + numdata; i++)
  //  {
  //  	printf("%0x\t", data_[i]);
  //  }
  //  std::cout << std::endl;

  size_t sentdata_size = boost::asio::write(serial_, boost::asio::buffer(data_, 5 + numparam));

  if (sentdata_size == (size_t)0)
    return false;
  return true;
}
std::vector<uint8_t> M3::ReceiveData(uint8_t& command)
{
  uint8_t start_count = 0;
  bool got_data = false;

  boost::array<uint8_t, 128> raw_bytes;
  std::vector<uint8_t> data;

  while (!got_data)
  {
    // Wait until first data sync of frame: 0x06, 0x85
    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
    if (start_count == 0)
    {
      if (raw_bytes[start_count] == 0x06)  // raw_bytes[0]
      {
        start_count = 1;
      }
    }
    else if (start_count == 1)
    {
      if (raw_bytes[start_count] == 0x85)  // raw_bytes[1]
      {
        start_count = 2;
      }
    }
    else if (start_count == 2)
    {
      if (raw_bytes[start_count] != 0)  // raw_bytes[2]: rx_len (command(1) + numdata(n))
      {
        uint8_t rx_len = raw_bytes[2];
        uint8_t numdata_ = rx_len - 1;

        // Now that entire start sequence has been found, read in the rest of the message
        got_data = true;

        boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[3], rx_len + 1));

        // seem to have got whole message
        // last uint8_t is CS
        uint8_t checkCS = rx_len;
        for (int i = 0; i < rx_len; i++)
        {
          checkCS ^= raw_bytes[3 + i];
        }

        if (checkCS == raw_bytes[3 + rx_len])
        {
          // CS good
          command = raw_bytes[3];
          for (int i = 0; i < numdata_; i++)
          {
            data.push_back(raw_bytes[4 + i]);
          }

          // 					for (int i = 0; i < 4 + rx_len; i++)
          // 					{
          // 						printf("%0x\t", raw_bytes[i]);
          // 					}
          // 					std::cout << std::endl;

          return data;
        }
        else
        {
          // Failed to check checksum. Return empty vector.
          return std::vector<uint8_t>();
        }
      }
      start_count = 0;
    }
  }

  return std::vector<uint8_t>();  //  Return empty vector.
}

std::vector<uint8_t> M3::Word2Bytes(int16_t dat)
{
  std::vector<uint8_t> retBytes(2);
  for (int i = 0; i < 2; i++)
    retBytes[1 - i] = (dat >> (i * 8));
  return retBytes;
}
std::vector<uint8_t> M3::Long2Bytes(int32_t dat)
{
  std::vector<uint8_t> retBytes(4);
  for (int i = 0; i < 4; i++)
    retBytes[3 - i] = (dat >> (i * 8));
  return retBytes;
}
int16_t M3::Bytes2Word(uint8_t* data)
{
  int dat_1 = data[0] << 8;
  int dat_2 = data[1];
  int dat = dat_1 | dat_2;
  return dat;
}
int32_t M3::Bytes2Long(uint8_t* data)
{
  int dat_1 = data[0] << 24;
  int dat_2 = data[1] << 16;
  int dat_3 = data[2] << 8;
  int dat_4 = data[3];
  int dat = dat_1 | dat_2 | dat_3 | dat_4;
  return dat;
}

}  // namespace isr_robot
