#include "isr_m3.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "isr2_m2_driver_node");

    ros::NodeHandle nh;

    std::shared_ptr<isr_m3_driver::isr_m3> isr_m3 = isr_m3_driver::isr_m3::create(nh);
    
    std::string port;
    int baudrate;
    if (!nh.getParam("port", port)) nh.param<std::string>("port", port, "/dev/ttyACM0");
    if (!nh.getParam("baudrate", baudrate)) nh.param<int>("baudrate", baudrate, 115200);
    
    if (!isr_m3->ConnectRobot(port, baudrate)) return 1;

    std::cout << "OK Ready" << std::endl;
    
    ros::spin();
    return 0;
}
