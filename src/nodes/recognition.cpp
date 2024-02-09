#include "ros/ros.h"
#include "procedural/RosInterface.h"

#include "procedural/utils/Parameters.h"

int main(int argc, char** argv)
{
    std::cout << "<<<<<<<<<<<<<<<<<< Launching Action Recognition <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    ros::init(argc, argv, "ActionRecognition");
    ros::NodeHandle n;
    auto onto_manipulators = onto::OntologiesManipulator();
    auto time_manipulators = mementar::TimelinesManipulator(&n);

    procedural::Parameters params;
    params.insert(procedural::Parameter("name", {"-n", "--name"}));
    params.insert(procedural::Parameter("action_path", {"-a", "--action_path"}));
    params.insert(procedural::Parameter("domain_path", {"-d", "--domain_path"}, {""}));
    params.insert(procedural::Parameter("ttl", {"-t", "--ttl"}, {"25"}));
    params.insert(procedural::Parameter("max_size", {"-s", "--max_size"}, {"500"}));

    params.set(argc, argv);
    params.display();

    procedural::RosInterface ros_interface(&n, onto_manipulators, time_manipulators, params.at("name").getFirst());
    if (ros_interface.init(params.at("action_path").getFirst(), stod(params.at("ttl").getFirst()),
                           stoi(params.at("max_size").getFirst()), params.at("domain_path").getFirst()))
        ros_interface.run();
    return 0;
}
