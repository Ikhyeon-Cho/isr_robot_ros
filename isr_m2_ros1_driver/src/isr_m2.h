#define _ISR_M2_H
#ifdef _ISR_M2_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <isr_m2_ros1_driver/RobotStatusStamped.h>
#include <isr_m2_ros1_driver/RobotCommand.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#define WHEEL_RADIUS_M 0.155	// Unit:m
#define WHEEL_BASE_M 0.531 // Unit:m
#define WHEEL_WIDTH_M 0.102 // Unit:m
#define ENCODER_PPR 6400.0	// Unit: pulse/rev
#define GEAR_RATIO 31.778	// Gearhead reduction ratio: 26 (26:1), Spurgear reduction ratio: 1.22 (44:36)
#define MPS2RPM 61.608 // same as (60 /(2 * M_PI * WHEEL_RADIUS_M))
#define MAX_RPM 4650.0

using namespace std::chrono_literals;

namespace isr_m2_driver
{

class ISR_M2
{
public:
    const ISR_M2 & operator=(const ISR_M2 &) = delete;
    ISR_M2(const ISR_M2 &) = delete;

    static std::shared_ptr<ISR_M2> create(ros::NodeHandle& nh)
    {
        auto isr_m2_shared_ptr = std::shared_ptr<ISR_M2>(new ISR_M2(nh));
        isr_m2_shared_ptr->weak_self_ = isr_m2_shared_ptr;
        return isr_m2_shared_ptr;
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

	bool SetVelocity(double leftWheelVel_MPS, double rightWheelVel_MPS);// m/s, m/s
	bool SetVelocityVW(double linearVel_MPS, double angularVel_RPS);// m/s, rad/s
	bool ReadVelocity_ADCApprox();

	bool ReadEncoder(void);
	bool ResetRobotPos(void);

	bool ReadRobotStatus(uint8_t& motorEnableStatus, uint8_t& motorStopStatus, uint8_t& emergencyButtonPressed);

public:
	ros::Time prev_encoder_time_; // ms
	ros::Time cur_encoder_time_; // ms

	long left_encoder_;	// A encoder value of left wheel (Unit: pulse)
	long right_encoder_;	// A encoder value of right wheel (Unit: pulse)

	double del_dist_left_m_; // left_encoder_ - prev_leftEncoder
	double del_dist_right_m_; // right_encoder_ - prev_rightEncoder

	double left_wheel_vel_endoer_mps_;
	double right_wheel_vel_encoder_mps_;

	bool left_wheel_direction_reading_;
	bool right_wheel_direction_reading_;
	double left_wheel_vel_reading_mps_;
	double right_wheel_vel_reading_mps_;

	struct Position
	{
		double x;
		double y;
		double theta;
	} position_; // Robot pose calculated by dead-reckoning (Unit: m, m, rad)

private:
    std::weak_ptr<ISR_M2> weak_self_;
	ros::NodeHandle nh_;
	ISR_M2(ros::NodeHandle& nh)
	: nh_(nh), serial_(io)
	{
		// Initialize cmd_vel msg subscription
		cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &ISR_M2::cmd_vel_callback, this);
		cmd_vel_msg_.linear.x;
		cmd_vel_msg_.angular.z;
		
		// Initialize odometry msg publisher
		odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);

		// Initialize robot status msg publisher
		robot_status_pub_ = nh_.advertise<isr_m2_ros1_driver::RobotStatusStamped>("robot_status", 10);
		robot_status_msg_.header.stamp = ros::Time::now();
		robot_status_msg_old_.header.stamp = ros::Time::now();

		// Initialize robot command service server
		robot_cmd_srv_ = nh_.advertiseService("robot_cmd", &ISR_M2::robot_cmd_callback, this);

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
            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.linear.z = 0;
            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = 0;
            odom.twist.covariance.fill(0);
            this->odom_pub_.publish(odom);
			
			// Read robot status
			static bool robot_status_changed = false;
			ReadRobotStatus(this->robot_status_msg_.motor_enabled, this->robot_status_msg_.motor_stopped, this->robot_status_msg_.estop_pressed);
			// ROS_INFO_STREAM("timer cb: read status >> "
			// 	<< "motor_enabled: "<< this->robot_status_msg_.motor_enabled
			// 	<< " / motor_stopped: " << this->robot_status_msg_.motor_stopped
			// 	<< " / estop_pressed: " << this->robot_status_msg_.estop_pressed);

			if (this->robot_status_msg_old_.motor_enabled != this->robot_status_msg_.motor_enabled) {
                ROS_INFO("Motor is %s", (this->robot_status_msg_.motor_enabled ? "ON" : "OFF"));
                robot_status_changed = true;
			}
			if (this->robot_status_msg_old_.motor_stopped != this->robot_status_msg_.motor_stopped) {
                ROS_INFO("Motor is %s", (this->robot_status_msg_.motor_stopped ? "Stopped" : "Resumed"));
                robot_status_changed = true;
			}
			if (this->robot_status_msg_old_.estop_pressed != this->robot_status_msg_.estop_pressed) {
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
	}

	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr msg)
	{
		cmd_vel_msg_ = *msg;
		// ROS_INFO_STREAM("Received cmd_vel << "
		// 	<< "v: " << this->cmd_vel_msg_->linear.x
		// 	<< " / w: " << this->cmd_vel_msg_->angular.z);
	}

	bool robot_cmd_callback(isr_m2_ros1_driver::RobotCommand::Request& request,
							isr_m2_ros1_driver::RobotCommand::Response& response)
	{
		while (ros::ok()) {
			if (this->serial_io_mut_.try_lock()) {
				response.result = isr_m2_ros1_driver::RobotCommand::Response::RESULT_FAIL;
				switch(request.command) {
					case isr_m2_ros1_driver::RobotCommand::Request::COMMAND_INITIALIZE:
						ROS_INFO("Initialize motor");
						if (Initialize()) response.result = isr_m2_ros1_driver::RobotCommand::Response::RESULT_SUCCESS;
						else ROS_ERROR("Initialize motor failed");
						break;
					case isr_m2_ros1_driver::RobotCommand::Request::COMMAND_ENABLE_MOTOR:
						if (request.val == 0) {
							ROS_INFO("Enable motor");
							if (EnableMotors()) response.result = isr_m2_ros1_driver::RobotCommand::Response::RESULT_SUCCESS;
							else ROS_ERROR("Enable motor failed");
						} else {
							ROS_INFO("Disable motor");
							if (DisableMotors()) response.result = isr_m2_ros1_driver::RobotCommand::Response::RESULT_SUCCESS;
							else ROS_ERROR("Disable motor failed");
						}
						break;
					case isr_m2_ros1_driver::RobotCommand::Request::COMMAND_STOP_MOTOR:
						if (request.val == 0) {
							ROS_INFO("Stop motor");
							if (StopMotors()) response.result = isr_m2_ros1_driver::RobotCommand::Response::RESULT_SUCCESS;
							else ROS_ERROR("Stop motor failed");
						} else {
							ROS_INFO("Resume motor");
							if (ResumeMotors()) response.result = isr_m2_ros1_driver::RobotCommand::Response::RESULT_SUCCESS;
							else ROS_ERROR("Resume motor failed");
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

		if (response.result == isr_m2_ros1_driver::RobotCommand::Response::RESULT_FAIL) return false;

		return true;
	}

	// serial io
	boost::asio::io_service io;
	boost::asio::serial_port serial_; /// @brief Actual serial port object for reading/writing to robot

	// serial io thread
	ros::WallTimer timer_;
	std::mutex serial_io_mut_;

	// cmd_vel subscription
    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::Twist cmd_vel_msg_;

	// odometry publisher
    ros::Publisher odom_pub_;

	// robot status publisher
	ros::Publisher robot_status_pub_;
	isr_m2_ros1_driver::RobotStatusStamped robot_status_msg_;
	isr_m2_ros1_driver::RobotStatusStamped robot_status_msg_old_;

	// robot command service server
    ros::ServiceServer robot_cmd_srv_;
	
	/**
	* @brief DeadReckoning
	* Do not need to use this func. 'SetVel' func calls this function.
	*/
	void DeadReckoning(long dl, long dr);	// Do not need to use this func. 'SetVel' func calls this function.

	bool SendData(uint8_t command, uint8_t numparam, uint8_t* params);
	std::vector<uint8_t> ReceiveData(uint8_t& command);

	std::vector<uint8_t> Word2Bytes(int16_t dat);
	std::vector<uint8_t> Long2Bytes(int32_t dat);
	int16_t Bytes2Word(uint8_t* data);
	int32_t Bytes2Long(uint8_t* data);
};
	
} // namespace isr_m2_driver

#endif // _ISR_M2_H