#ifndef MEMORYROSINTERFACE_H
#define MEMORYROSINTERFACE_H

#include <ros/ros.h>
#include <string>
#include <procedural/memory/ProceduralFullReader.h>
#include "procedural_interfaces/getRobotActions.h"
#include "procedural_interfaces/getPractices.h"
#include "procedural_interfaces/getPracticeFrames.h"
#include "procedural_interfaces/getRoles.h"
#include "procedural_interfaces/GetPriorities.h"
#include "procedural_interfaces/GetTasks.h"
#include "procedural_interfaces/GetActions.h"
#include "procedural_interfaces/GetTaskDetails.h"
#include "procedural_interfaces/GetActionDetails.h"



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

    bool getPriorities(procedural_interfaces::GetPriorities::Request &req,
                      procedural_interfaces::GetPriorities::Response &res);

    bool getTasks(procedural_interfaces::GetTasks::Request &req,
                  procedural_interfaces::GetTasks::Response &res);

    bool getActions(procedural_interfaces::GetActions::Request &req,
                    procedural_interfaces::GetActions::Response &res);

    bool getTaskDetails(procedural_interfaces::GetTaskDetails::Request &req,
                        procedural_interfaces::GetTaskDetails::Response &res);

    bool getActionDetails(procedural_interfaces::GetActionDetails::Request &req,
                          procedural_interfaces::GetActionDetails::Response &res);










private:
    void init_ros();
    void getRobotActions();

    ros::NodeHandle* node_;
    ros::ServiceServer getRobotActions_service_;
    ros::ServiceServer getPractices_service_;
    ros::ServiceServer getPracticeFrames_service_;
    ros::ServiceServer getRoles_service_;
    ros::ServiceServer getPriorities_service_;
    ros::ServiceServer getTasks_service_;
    ros::ServiceServer getActions_service_;
    ros::ServiceServer getTaskDetails_service_;
    ros::ServiceServer getActionDetails_service_;
    ProceduralFullReader parser_;
    bool run_;

};

} // procedural

#endif //MEMORYROSINTERFACE_H
