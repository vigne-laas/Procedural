#ifndef PROCEDURAL_HTNBUILDER_H
#define PROCEDURAL_HTNBUILDER_H
#include "procedural/reader/domainTypes/ParsedHTN.h"
#include "procedural/core/Types/ActionType.h"
#include <vector>
namespace procedural {

class HTNBuilder
{
public:
    HTNBuilder() = default;
    void buildAction(const std::vector<PrimitiveActionParsed_t>& actions);
    void buildTask(const std::vector<Abstract_task_t>& abstract_tasks, const std::vector<PrimitiveActionParsed_t>& actions_htn,
                   const std::vector<ActionType*>& actions_type);
    void displayActions();
private:
    std::vector<Task*> tasks_;
    std::vector<std::string> actions_possible;

    bool checkActions(const std::vector<PrimitiveActionParsed_t>& actions_htn, const std::vector<ActionType*>& action_type);
};

} // procedural

#endif //PROCEDURAL_HTNBUILDER_H
