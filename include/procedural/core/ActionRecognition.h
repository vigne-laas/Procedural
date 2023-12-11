#ifndef PROCEDURAL_ACTIONRECOGNITION_H
#define PROCEDURAL_ACTIONRECOGNITION_H

#include <vector>
#include <queue>
#include <mutex>
#include "procedural/core/Types/Fact.h"
#include "procedural/utils/BufferFacts.h"
#include "procedural/core/Types/StateMachineFinishedMSG_.h"

namespace procedural {

class Action;

class ActionRecognition : public ISubject
{
public:

    ActionRecognition() : buffer_(nullptr) {};
    void init(const std::vector<Action*>& actions, double tll = 25, int max_size = 500);


    void addToQueue(Fact* fact) const;

    void processQueue(TimeStamp_t current_time);

    void setCallback(const std::function<void(
            const std::vector<StateMachineFinishedMSG_>&)>& callback) { callback_output_ = callback; }

    void linkToTaskRecognition(IObserver* task_recognition);
    void attach(IObserver* observer) override;
    void detach(IObserver* observer) override;
    void notify(MessageType type) override;
private:
    static void defaultCallback(const std::vector<StateMachineFinishedMSG_>& outputs);
    BufferFacts* buffer_{};
    std::vector<Action*> actions_;

    std::function<void(const std::vector<StateMachineFinishedMSG_>&)> callback_output_;
    std::vector<Action*> finished_actions_;
    std::vector<Action*> updated_actions_;

    std::list<IObserver*> recognition_process_observer_;


    void send_updated_action_(std::vector<StateMachineFinishedMSG_> new_info_action);
};

} // namespace procedural

#endif //PROCEDURAL_ACTIONRECOGNITION_H
