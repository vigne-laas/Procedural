#ifndef PROCEDURAL_HTNBUILDER_H
#define PROCEDURAL_HTNBUILDER_H
#include "procedural/reader/domainTypes/ParsedHTN.h"
#include "procedural/core/Types/ActionMethod.h"
#include <vector>
namespace procedural {

class HTNBuilder
{
public:
    HTNBuilder() = default;
    void buildAction(const std::vector<PrimitiveActionParsed_t>& actions);
    void buildTask(const std::vector<Abstract_task_t>& abstract_tasks,
                   const std::vector<PrimitiveActionParsed_t>& actions_htn,
                   const std::vector<ActionMethod*>& actions_type);
    void displayTask();
    std::vector<Task*> getTask() { return tasks_; }
private:
    std::vector<Task*> tasks_;
    std::map<std::string, TransitionType> actions_possible;

    bool checkActions(const std::vector<PrimitiveActionParsed_t>& actions_htn,
                      const std::vector<ActionMethod*>& action_type);
};

} // procedural

#endif //PROCEDURAL_HTNBUILDER_H
