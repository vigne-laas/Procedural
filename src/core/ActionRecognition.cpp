#include <set>
#include "procedural/core/Types/ActionMethod.h"
#include "procedural/core/ActionRecognition.h"
#include "procedural/core/Types/ResultFeedProcess.h"
#include <iostream>

namespace procedural {
void ActionRecognition::init(const std::vector<ActionMethod*>& actions, double ttl, int max_size)
{
    actions_ = actions;
    callback_output_ = ActionRecognition::defaultCallback;
    double max_ttl = 0;
    for (auto& action: actions_)
    {
//        action->attachRecognitionProcess(this);
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
        ResultFeedProcess_t result;
        for (auto& action: actions_)
        {
            result.combine(action->feed(fact, current_time));
            if (result.evolve)
                facts_used.insert(fact->getId());
        }

        int nb_update;
        do
        {
            nb_update = 0;
            std::vector<StateMachineFinishedMSG_> msg_finished_SM_;
            std::vector<ActionMethod*> finished_actions = result.finished_actions;
            result.finished_actions.clear();

            for (auto& finished_action: finished_actions)
            {
                std::cout << "finished action : " << finished_action->getName() << std::endl;
                std::set<uint32_t> temp_set = finished_action->checkCompleteStateMachines(current_time);
                set_id_facts.insert(temp_set.begin(), temp_set.end());
                nb_update++;
                for (auto& action: actions_)
                    if (action != finished_action)
                    {
                        result.combine(action->checkSubAction(finished_action));
                        if (result.evolve)
                            nb_update++;
                    }

                auto complete_state_machines = finished_action->getCompletesStateMachines();
                for (auto& complete_state_machine: complete_state_machines)
                    msg_finished_SM_.emplace_back(complete_state_machine);

            }
            for (auto& updated_action: result.updated_actions)
            {
                auto nets = updated_action->getNewExplanation();
                for (auto& net: nets)
                    msg_finished_SM_.emplace_back(net, true);
            }
            result.updated_actions.clear();


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

void ActionRecognition::linkToTaskRecognition(IObserver* task_recognition)
{
    for (const auto& action: actions_)
        action->attachRecognitionProcess(task_recognition);
}


} // namespace procedural