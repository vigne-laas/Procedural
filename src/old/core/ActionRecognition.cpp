#include <set>
#include "procedural/old/core/Types/Action.h"
#include "procedural/old/core/ActionRecognition.h"
#include "procedural/old/core/Types/ResultFeedProcess.h"
#include <iostream>

namespace procedural {
void ActionRecognition::init(const std::vector<Action*>& actions, double ttl, int max_size)
{
    actions_ = actions;
    callback_output_ = ActionRecognition::defaultCallback;
    double max_ttl = 0;
    for (auto& action: actions_)
    {
//        action->attachRecognitionProcess(this);
        if (action->getTtl() > max_ttl)
            max_ttl = action->getTtl();
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
        ResultFeedProcess_t<Action> result;
        for (auto& action: actions_)
        {
            auto feed_result = action->feed(fact, current_time);
            result.combine(feed_result, action);
            if (feed_result.state >= FeedResult::EVOLVE)
                facts_used.insert(fact->getId());
        }

        int nb_update;
        do
        {
            nb_update = 0;
            std::vector<StateMachineFinishedMSG_> msg_finished_SM_;
            std::vector<ActionEvent_t> actions_events_;
            std::vector<Action*> finished_actions = result.finished_;
            result.finished_.clear();

            for (auto& finished_action: finished_actions)
            {
                std::cout << "finished action : " << finished_action->getName() << std::endl;
                std::set<uint32_t> temp_set = finished_action->checkStateMachine(current_time);
                set_id_facts.insert(temp_set.begin(), temp_set.end());
                nb_update++;
                for (auto& action: actions_)
                    if (action != finished_action)
                    {
                        result.combine(action->checksubAction(finished_action), action);
                        if (result.evolve)
                            nb_update++;
                    }

                auto complete_state_machines = finished_action->getFinishedStateMachine();
                for (auto& complete_state_machine: complete_state_machines)
                {
                    msg_finished_SM_.emplace_back(complete_state_machine);
                    actions_events_.emplace_back(FeedResult::FINISH, complete_state_machine);
                }


            }
            for (auto& updated_action: result.updated_)
            {
                auto nets = updated_action->getNewExplanation();
                for (auto& net: nets)
                {
                    msg_finished_SM_.emplace_back(net, true);
                    actions_events_.emplace_back(FeedResult::NEW_EXPLANATION, net);
                }

            }
            result.updated_.clear();


            callback_output_(msg_finished_SM_);
            sendActionEvent_(actions_events_);

            if (nb_update != 0)
                for (auto& action: actions_)
                    action->cleanInvolve(set_id_facts);

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
    attach(task_recognition);
}
void ActionRecognition::attach(IObserver* observer)
{
    recognition_process_observer_.push_back(observer);
}
void ActionRecognition::detach(IObserver* observer)
{
    recognition_process_observer_.remove(observer);
}
void ActionRecognition::notify(ActionEvent_t event)
{
    for (const auto& obs: recognition_process_observer_)
        obs->actionEvent(event);
}
void ActionRecognition::sendActionEvent_(const std::vector<ActionEvent_t>& new_info_action)
{
    for (const auto& msg: new_info_action)
        notify(msg);
}


} // namespace procedural