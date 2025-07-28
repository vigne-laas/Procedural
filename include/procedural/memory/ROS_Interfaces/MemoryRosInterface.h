#ifndef MEMORYROSINTERFACE_H
#define MEMORYROSINTERFACE_H

#include <ros/ros.h>
#include <string>
#include <procedural/memory/ProceduralFullReader.h>
#include "procedural_interfaces/getRobotActions.h"
#include "procedural_interfaces/getPractices.h"
#include "procedural_interfaces/getPracticeFrames.h"
#include "procedural_interfaces/getRoles.h"



namespace procedural {

class MemoryROSInterface {
public :
    MemoryROSInterface(ros::NodeHandle* n, const std::string& path = "");

    bool init(const std::string& path);
    void run() const;
    void stop() {run_ = false;}
    inline bool isRunning() const {return run_;}

    bool getRobotActions(procedural_interfaces::getRobotActions::Request &req,
                         procedural_interfaces::getRobotActions::Response &res);

    bool getPractices(procedural_interfaces::getPractices::Request &req,
                             procedural_interfaces::getPractices::Response &res);

    bool getPracticeFrames(procedural_interfaces::getPracticeFrames::Request &req,
                           procedural_interfaces::getPracticeFrames::Response &res);
    bool getRoles(procedural_interfaces::getRoles::Request &req,
                  procedural_interfaces::getRoles::Response &res);










private:
    void init_ros();
    void getRobotActions();

    ros::NodeHandle* node_;
    ros::ServiceServer getRobotActions_service_;
    ros::ServiceServer getPractices_service_;
    ros::ServiceServer getPracticeFrames_service_;
    ros::ServiceServer getRoles_service_;
    ProceduralFullReader parser_;
    bool run_;

};

} // procedural

#endif //MEMORYROSINTERFACE_H
