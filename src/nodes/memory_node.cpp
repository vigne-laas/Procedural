#include <procedural/memory/ProceduralReader.h>
#include <procedural/memory/ROS_Interfaces/MemoryRosInterface.h>

#include "ros/ros.h"
#include "procedural/old/utils/Parameters.h"

int main(int argc, char** argv)
{
    std::cout << "<<<<<<<<<<<<<<<<<< Launching Procedurual Memory  <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    ros::init(argc, argv, "ProceduralMemory");
    ros::NodeHandle n;

    ::procedural::Parameters params;
    params.insert(::procedural::Parameter("name", {"-n", "--name"}));
    params.insert(::procedural::Parameter("file_path", {"-f", "--file_path"}));

    params.set(argc, argv);
    params.display();
    ::procedural::MemoryROSInterface memory_ros_interface(&n, params.at("file_path").getFirst());
    memory_ros_interface.run();


    // procedural::RosInterface ros_interface(&n, onto_manipulators, time_manipulators, params.at("name").getFirst());
    // if (ros_interface.init(params.at("action_path").getFirst(), stod(params.at("ttl").getFirst()),
    //                        stoi(params.at("max_size").getFirst()), params.at("domain_path").getFirst()))
    //     ros_interface.run();
    return 0;
}
