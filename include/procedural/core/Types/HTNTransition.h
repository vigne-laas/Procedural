#ifndef PROCEDURAL_HTNTRANSITION_H
#define PROCEDURAL_HTNTRANSITION_H

namespace procedural
{
enum class TransitionType{
    None,
    Action,
    Task
};

struct HTNTransition_t
{
    TransitionType type;
    int step;
    int id_subtask_parsed;
    uint32_t id_subtask;
    std::vector<int> id_contraints_order;
    std::map<std::string,std::string> arguments_;

};

}//procedural

#endif //PROCEDURAL_HTNTRANSITION_H
