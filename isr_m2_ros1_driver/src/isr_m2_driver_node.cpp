#include "isr_m2.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "isr2_m2_driver_node");

    ros::NodeHandle nh;

    std::shared_ptr<isr_m2_driver::ISR_M2> isr_m2 = isr_m2_driver::ISR_M2::create(nh);
    
    std::string port;
    int baudrate;
    if (!nh.getParam("port", port)) nh.param<std::string>("port", port, "/dev/ttyACM0");
    if (!nh.getParam("baudrate", baudrate)) nh.param<int>("baudrate", baudrate, 115200);
    
    if (!isr_m2->ConnectRobot(port, baudrate)) return 1;

    std::cout << "OK Ready" << std::endl;
    
    ros::spin();
    return 0;
}
