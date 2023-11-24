
#include "procedural/core/Types/Method.h"

namespace procedural {
Method::Method(const std::string& name, int id) : name_(name), id_(id), compt_SM_(0)
{
    factory_machine_ = new StateMachine(name, id);
}

void Method::addTransition(const HTNTransition_t& transition)
{
    factory_machine_->addTransition(transition);
}
void Method::close()
{
    factory_machine_->closeStateMachine();
}
std::string Method::getStrStructure()
{
    std::string res;
    res += "Structure of : " + factory_machine_->getName() + "\n\n";
    res += factory_machine_->getStrStructure() + "\n";
    return res;
}
bool Method::feed(Action* action)
{
    bool evolve = false;
    for (auto& state_machine: current_state_machines_)
    {
        if (state_machine->evolve(action))
        {
            std::cout << "\t succes of evolution  : " << state_machine->getName() << std::endl;
            evolve = true;
        }
    }

    if (evolve == false)
    {
        StateMachine* new_net = factory_machine_->clone(-1);
        if (new_net->evolve(action))
        {
            new_net->setId(getNextSMId());
            current_state_machines_.insert(new_net);
            std::cout << "create new state machine " << new_net->getName() << std::endl;

            evolve = true;
        } else
            delete new_net;
    }

    return evolve;
}
bool Method::feed(Task* task)
{
    bool evolve = false;
    for (auto& state_machine: current_state_machines_)
    {
        if (state_machine->evolve(task))
        {
            std::cout << "\t succes of evolution  : " << state_machine->getName() << std::endl;
            evolve = true;
        }
    }

    if (evolve == false)
    {
        StateMachine* new_net = factory_machine_->clone(-1);
        if (new_net->evolve(task))
        {
            new_net->setId(getNextSMId());
            current_state_machines_.insert(new_net);
            std::cout << "create new state machine " << new_net->getName() << std::endl;

            evolve = true;
        } else
            delete new_net;
    }

    return evolve;
}
bool Method::feed(ActionMethod* action_method)
{
    return false;
}
void Method::updateStateMachine(MessageType type, StateMachine* machine)
{
    std::cout << "receive info from SM : " << machine->getName() << " : " << std::to_string((int) type)
              << "in method : " << name_ << std::endl;
    if (type == MessageType::Update)
        updated_states_machines.push_back(machine);
    if (type == MessageType::Complete or type == MessageType::Finished)
        finished_state_machines_.insert(machine);

    notify(type);
}
void Method::notify(MessageType type)
{
    for (const auto& obs: list_observer_)
        obs->updateMethod(type, this);
}
} // procedural