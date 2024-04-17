#include "isr_m3.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "isr_m3");
  ros::NodeHandle nh{ "isr_m3" };

  auto isr_m3 = isr_robot::M3::create(nh);
  std::string port;
  int baudrate{ 115200 };

  if (nh.getParam("port", port))
    nh.param<std::string>("port", port, "/dev/ttyACM0");

  if (!isr_m3->ConnectRobot(port, baudrate))
  {
    ros::shutdown();
    return 1;
  }

  // Make output green
  std::cout << "\033[1;32m" << "==> ISR-M3 successfully connected!" << "\033[0m" << std::endl;

  ros::spin();
  return 0;
}
