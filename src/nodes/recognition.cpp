#include "ros/ros.h"
#include "procedural/RosInterface.h"

int main(int argc, char** argv)
{
    std::cout << "<<<<<<<<<<<<<<<<<< Launching Action Recognition <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    ros::init(argc, argv, "ActionRecognition");
    ros::NodeHandle n;
    procedural::RosInterface ros_interface(&n);
    ros_interface.run();
    return 0;
}
