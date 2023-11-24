#ifndef PROCEDURAL_ACTIONRECOGNITION_H
#define PROCEDURAL_ACTIONRECOGNITION_H

#include <vector>
#include <queue>
#include <mutex>
#include "procedural/core/Types/Fact.h"
#include "procedural/utils/BufferFacts.h"
#include "procedural/core/Types/StateMachineFinishedMSG_.h"

namespace procedural {

class ActionMethod;

class ActionRecognition :
{
public:

    ActionRecognition() : buffer_(nullptr) {};
    void init(const std::vector<ActionMethod*>& actions, double tll = 25, int max_size = 500);


    void addToQueue(Fact* fact) const;

    void processQueue(TimeStamp_t current_time);

    void setCallback(const std::function<void(
            const std::vector<StateMachineFinishedMSG_>&)>& callback) { callback_output_ = callback; }

    void linkToTaskRecognition(IObserver* task_recognition);
private:
    static void defaultCallback(const std::vector<StateMachineFinishedMSG_>& outputs);
    BufferFacts* buffer_{};
    std::vector<ActionMethod*> actions_;

    std::function<void(const std::vector<StateMachineFinishedMSG_>&)> callback_output_;
    std::vector<ActionMethod*> finished_actions_;
    std::vector<ActionMethod*> updated_actions_;

};

} // namespace procedural

#endif //PROCEDURAL_ACTIONRECOGNITION_H
