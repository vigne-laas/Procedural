#ifndef PROCEDURAL_HTNBUILDER_H
#define PROCEDURAL_HTNBUILDER_H

#include "procedural/task_recognition/Reader/domainTypes/ParsedHTN.h"
#include "procedural/task_recognition/Structures/Task.h"

namespace procedural {
class Action;
class HTNBuilder {
public:
    HTNBuilder() = default;

    explicit HTNBuilder(const HTNParserd_t& htn) : htn_(htn) {};
    bool build(const HTNParserd_t& htn);
    bool checkAction(const std::vector<Action*>& actions);
    bool build(const HTNParserd_t& htn,const std::vector<Action*>& actions);
    HTNParserd_t getHTN() { return htn_; }
    std::vector<Task*> getTasks() { return tasks_; }


private:
    HTNParserd_t htn_;
    std::vector<Task*> tasks_;

};

} // procedural

#endif //PROCEDURAL_HTNBUILDER_H
