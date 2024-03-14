#ifndef PROCEDURAL_HTNTRANSITION_H
#define PROCEDURAL_HTNTRANSITION_H

namespace procedural {
enum class TransitionType
{
    None,
    Action,
    ActionMethod,
    Task
};

struct HTNTransition_t
{
    TransitionType type;
    int step;
    int id_subtask_parsed;
    uint32_t id_subtask;
    std::vector<int> id_contraints_order;
    std::map<std::string, std::string> arguments_;

    std::string toString() const
    {
        std::string msg = "T : ";
        if (type == TransitionType::ActionMethod)
            msg += "ActionMethod";
        if (type == TransitionType::Action)
            msg += "Action";
        if (type == TransitionType::Task)
            msg += "Task";
        if (type == TransitionType::None)
            msg += "None";

        msg += " ActionId:" + std::to_string(id_subtask_parsed);
        msg += " id contrains transition:" + std::to_string(id_subtask);
        msg += " step:" + std::to_string(step);
        return msg;
    }

};


}//procedural

#endif //PROCEDURAL_HTNTRANSITION_H
