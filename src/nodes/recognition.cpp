#include "ros/ros.h"
#include "procedural/RosInterface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ActionRecognition");
    ros::NodeHandle n("ActionRecognition");
    procedural::RosInterface ros_interface(&n);
    ros_interface.run();
    return 0;
}
