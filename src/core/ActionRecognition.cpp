#include <set>
#include "procedural/core/Types/ActionType.h"
#include "procedural/utils/ActionRecognition.h"
#include <iostream>

namespace procedural {
ActionRecognition::ActionRecognition(const std::vector<ActionType*>& actions, double ttl, int max_size) : actions_(
        actions), callback_output_(ActionRecognition::defaultCallback)
{
    double max_ttl = 0;
    for (auto& action: actions_)
        if (action->maxTtl() > max_ttl)
            max_ttl = action->maxTtl();
    if (ttl > max_ttl)
        max_ttl = ttl;
    buffer_ = new BufferFacts(max_ttl, max_size);
}

void ActionRecognition::addToQueue(Fact* fact) const
{
    buffer_->addFact(fact);
}

void ActionRecognition::processQueue(TimeStamp_t current_time)
{
    std::vector<Fact*> list_facts = buffer_->getFacts(current_time);
    std::set<uint32_t> facts_used;

    for (auto fact: list_facts)
    {
         std::cout << "--------------" << std::endl;
         std::cout << "fact in Action recognition: " << fact->toString() << " " << fact->getTimeStamp() << "\n\n"
                   << std::endl;
        std::set<uint32_t> set_id_facts;
        std::vector<procedural::ActionType*> complete_actions;
        for (auto& action: actions_)
            if(action->feed(fact, current_time))
                facts_used.insert(fact->getId());

        int nb_update;
        do
        {
            nb_update = 0;
            std::vector<StateMachineFinishedMSG_> msg_finished_SM_; // TODO rename
            for (auto& action: actions_)
            {
                std::set<uint32_t> temp_set = action->checkCompleteStateMachines(current_time);
                if (temp_set.empty() == false)
                {
                    set_id_facts.insert(temp_set.begin(), temp_set.end());
                    complete_actions.push_back(action);
                    nb_update++;
                }
                else
                {
                    if (action->checkNewExplanation())
                    {
                        auto nets = action->getNewExplanation();
                        for (auto& net: nets)
                            msg_finished_SM_.emplace_back(net, true);
                    }
                }
            }

            for (auto& action_complete: complete_actions)
            {
                for (auto& action: actions_)
                    if (action != action_complete)
                    {
                        if (action->checkSubAction(action_complete))
                        {
                            if (action->checkNewExplanation())
                            {
                                auto nets = action->getNewExplanation();
                                for (auto& net: nets)
                                    msg_finished_SM_.emplace_back(net, true);
                            }
                            nb_update++;
                        }
                    }

                auto complete_state_machines = action_complete->getCompletesStateMachines();
                for (auto& complete_state_machine: complete_state_machines)
                    msg_finished_SM_.emplace_back(complete_state_machine);
            }

            callback_output_(msg_finished_SM_);

            if (nb_update != 0)
                for (auto& action: actions_)
                    action->cleanActions(set_id_facts);

            complete_actions.clear();
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

} // namespace procedural