#ifndef PROCEDURAL_TRANSITIONTASK_H
#define PROCEDURAL_TRANSITIONTASK_H

#include <map>
#include <string>
#include <ontologenius/clients/ontologyClients/IndividualClient.h>

#include "procedural/core/Types/Variable.h"

namespace procedural {
class Task;
class TransitionTask
{
public:
    explicit TransitionTask(uint32_t task_id, int next_state, const std::map<std::string, std::string>& arguments);
    TransitionTask(const TransitionTask& t,int next_state);



    void linkVariables(std::map<std::string, Variable_t>& variables);

    bool match(Task* task);


    std::string toString() const;
    void setOntologyClient(onto::IndividualClient* indiv_client);
private:
    uint32_t task_id_;
    std::map<std::string,Variable_t*> variables_;
    int id_next_state_;
    std::map<std::string, std::string> arguments_;
    bool checkArgs(Task* task);
};

} // procedural

#endif //PROCEDURAL_TRANSITIONTASK_H
