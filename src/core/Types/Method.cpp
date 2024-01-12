
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
EvolveResult_t Method::feed(StateMachine* state_machine_finished)
{
    EvolveResult_t res;
    bool evolve = false;
    for (auto& state_machine: current_state_machines_)
    {
        std::cout << "check if " << state_machine_finished->getName() << " can evolve method : " << name_;
        auto result = state_machine->evolve(state_machine_finished);
        switch (result.state)
        {

            case FeedResult::NO_EVOLUTION:
                break;
            case FeedResult::EVOLVE:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << std::endl;
                evolve = true;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << "reach final state"
                          << std::endl;
                evolve = true;
                finished_state_machines_.insert(state_machine);
                res.state = FeedResult::FINISH;

                break;
        }
        if (result.update_available)
        {
            updated_states_machines.push_back(state_machine);
            res.update_available = true;
        }

    }
    if (evolve == false)
    {
//        std::cout << "Try to create new SM " << std::endl;
        StateMachine* new_net = factory_machine_->clone(-1);
        auto result = new_net->evolve(state_machine_finished);
        switch (result.state)
        {
            case FeedResult::NO_EVOLUTION:
                delete new_net;
                break;
            case FeedResult::EVOLVE:
                new_net->setId(getNextSMId());
//                    new_net->attach(this);
                current_state_machines_.insert(new_net);
                std::cout << "create new state machine " << new_net->getName() << std::endl;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                new_net->setId(getNextSMId());
                std::cout << "Method finish" << name_ << std::endl;
                finished_state_machines_.insert(new_net);
                break;
        }

    }
    return res;
}
EvolveResult_t Method::feed(Task* task)
{
    EvolveResult_t res;
    bool evolve = false;
    for (auto& state_machine: current_state_machines_)
    {
        auto res = state_machine->evolve(task);
        if (res.state >= FeedResult::EVOLVE)
        auto result = state_machine->evolve(task);
        switch (result.state)
        {

            case FeedResult::NO_EVOLUTION:
                break;
            case FeedResult::EVOLVE:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << std::endl;
                evolve = true;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                std::cout << "\t succes of evolution  : " << state_machine->getName() << "reach final state"
                          << std::endl;
                evolve = true;
                finished_state_machines_.insert(state_machine);
                res.state = FeedResult::FINISH;

                break;
        }
        if (result.update_available)
        {
            updated_states_machines.push_back(state_machine);
            res.update_available = true;
        }
    }

    if (evolve == false)
    {
        StateMachine* new_net = factory_machine_->clone(-1);
        auto result = new_net->evolve(task);
        switch (result.state)
        {
            case FeedResult::NO_EVOLUTION:
                delete new_net;
                break;
            case FeedResult::EVOLVE:
                new_net->setId(getNextSMId());
//                    new_net->attach(this);
                current_state_machines_.insert(new_net);
                std::cout << "create new state machine " << new_net->getName() << std::endl;
                if (res.state != FeedResult::FINISH)
                    res.state = FeedResult::EVOLVE;
                break;
            case FeedResult::FINISH:
                new_net->setId(getNextSMId());
                finished_state_machines_.insert(new_net);
                break;
        }

            evolve = true;
    }

    return res;
}
        } else
            delete new_net;
    }

    return evolve;
}
//bool Method::feed(ActionMethod* action_method)
//{
//    return false;
//}
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