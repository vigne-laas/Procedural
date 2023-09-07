#include <set>
#include "procedural/core/Types/ActionMethod.h"
#include "procedural/core/ActionRecognition.h"
#include <iostream>

namespace procedural {
void ActionRecognition::init(const std::vector<ActionMethod*>& actions, double ttl, int max_size)
{
    actions_ = actions;
    callback_output_ = ActionRecognition::defaultCallback;
    double max_ttl = 0;
    for (auto& action: actions_)
    {
        action->attachRecognitionProcess(this);
        if (action->maxTtl() > max_ttl)
            max_ttl = action->maxTtl();
    }

    if (ttl > max_ttl)
        max_ttl = ttl;
    buffer_ = new BufferFacts(max_ttl, max_size);
}

void ActionRecognition::addToQueue(Fact* fact) const
{
    if (buffer_ != nullptr)
        buffer_->addFact(fact);
}

void ActionRecognition::processQueue(TimeStamp_t current_time)
{
    if (buffer_ == nullptr)
        return;

    std::vector<Fact*> list_facts = buffer_->getFacts(current_time);
    std::set<uint32_t> facts_used;

    for (auto fact: list_facts)
    {
        std::cout << "--------------" << std::endl;
        std::cout << "fact in Action recognition: " << fact->toString() << " " << fact->getTimeStamp() << "\n\n"
                  << std::endl;
        std::set<uint32_t> set_id_facts;
        for (auto& action: actions_)
            if (action->feed(fact, current_time))
                facts_used.insert(fact->getId());

        int nb_update;
        do
        {
            nb_update = 0;
            std::vector<StateMachineFinishedMSG_> msg_finished_SM_;
            std::vector<ActionMethod*> finished_actions = finished_actions_;
            std::vector<ActionMethod*> updated_actions = updated_actions_;
            finished_actions_.clear();
            updated_actions_.clear();
            for (auto& finished_action: finished_actions)
            {
                std::set<uint32_t> temp_set = finished_action->checkCompleteStateMachines(current_time);
                set_id_facts.insert(temp_set.begin(), temp_set.end());
                nb_update++;
                for (auto& action: actions_)
                    if (action != finished_action)
                        if (action->checkSubAction(finished_action))
                            nb_update++;

                auto complete_state_machines = finished_action->getCompletesStateMachines();
                for (auto& complete_state_machine: complete_state_machines)
                    msg_finished_SM_.emplace_back(complete_state_machine);
            }
            for (auto& updated_action: updated_actions)
            {
                auto nets = updated_action->getNewExplanation();
                for (auto& net: nets)
                    msg_finished_SM_.emplace_back(net, true);
            }


//            for (auto& action: actions_)
//            {
//                std::set<uint32_t> temp_set = action->checkCompleteStateMachines(current_time);
//                if (temp_set.empty() == false)
//                {
//                    set_id_facts.insert(temp_set.begin(), temp_set.end());
//                    complete_actions.push_back(action);
//                    nb_update++;
//                } else
//                {
//                    if (action->checkNewExplanation())
//                    {
//                        auto nets = action->getNewExplanation();
//                        for (auto& net: nets)
//                            msg_finished_SM_.emplace_back(net, true);
//                    }
//                }
//            }

//            for (auto& action_complete: complete_actions)
//            {
//                for (auto& action: actions_)
//                    if (action != action_complete)
//                    {
//                        if (action->checkSubAction(action_complete))
//                        {
//                            if (action->checkNewExplanation())
//                            {
//                                auto nets = action->getNewExplanation();
//                                for (auto& net: nets)
//                                    msg_finished_SM_.emplace_back(net, true);
//                            }
//                            nb_update++;
//                        }
//                    }
//
//                auto complete_state_machines = action_complete->getCompletesStateMachines();
//                for (auto& complete_state_machine: complete_state_machines)
//                    msg_finished_SM_.emplace_back(complete_state_machine);
//            }

            callback_output_(msg_finished_SM_);

            if (nb_update != 0)
                for (auto& action: actions_)
                    action->cleanActions(set_id_facts);

        } while (nb_update != 0);

        for (auto& action: actions_)
            action->clean();

        facts_used.insert(set_id_facts.begin(), set_id_facts.end());
    }

    buffer_->cleanUsedFacts(facts_used);
}

void ActionRecognition::defaultCallback(const std::vector<StateMachineFinishedMSG_>& outputs)
{
    for (const auto& output: outputs)
        std::cout << ">> " << output << std::endl;
}
void ActionRecognition::updateActionMethod(procedural::MessageType type, procedural::ActionMethod* action_method)
{
    std::cout << "new ActionMethod data in Action_Recognition" << std::to_string(static_cast<int>(type)) << std::endl;
    if (type == MessageType::Complete or type == MessageType::Finished)
    {
        finished_actions_.push_back(action_method);
    }
    if(type==MessageType::Update)
        updated_actions_.push_back(action_method);
}
void ActionRecognition::updateAction(MessageType type, Action* machine)
{
    std::cout << "new Action data in Action_Recognition" << std::to_string(static_cast<int>(type)) << std::endl;

//    if (type == MessageType::Complete or type == MessageType::Finished)
//    {
//        finished_actions_.
//
//    }
}

} // namespace procedural