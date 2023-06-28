#include "ros/ros.h"
#include "procedural/RosInterface.h"

int main(int argc, char** argv)
{
    std::cout << "<<<<<<<<<<<<<<<<<< Launching Action Recognition <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    ros::init(argc, argv, "ActionRecognition");
    ros::NodeHandle n;
    auto onto_manipulators = OntologiesManipulator(&n);
    auto time_manipulators = mementar::TimelinesManipulator(&n);
    bool var_name = false;
    std::string name;
    if (argc > 1)
        for (int i = 0; i < argc; i++)
        {
            std::string arg(argv[i]);
            if (var_name)
                name = arg;
            if (arg.find("-n") != std::string::npos)
                var_name = true;
            else
                var_name = false;
     }

    procedural::RosInterface ros_interface(&n, onto_manipulators, time_manipulators, name);
    ros_interface.run();
    return 0;
}
