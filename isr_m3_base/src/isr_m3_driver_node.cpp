#include "isr_m3.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "isr2_m2_driver_node");
  ros::NodeHandle nh;

  auto isr_m3 = isr_robot::M3::create(nh);
  std::string port{ nh.param<std::string>("port", port, "/dev/ttyACM0") };
  int baudrate{ nh.param<int>("baudrate", baudrate, 115200) };

  if (!isr_m3->ConnectRobot(port, baudrate))
  {
    ros::shutdown();
    return 1;
  }

  std::cout << "OK Ready" << std::endl;

  ros::spin();
  return 0;
}
