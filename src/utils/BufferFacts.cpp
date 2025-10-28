#include <set>
#include "procedural/utils/BufferFacts.h"
#include "procedural/utils/Logger.h"
#include <algorithm>
#include <iostream>

namespace procedural {

BufferFacts::BufferFacts(double ttl, int max_size) :
        read_queue(&primary_queue),  write_queue(&secondary_queue),
        ttl_(ttl), max_size_(max_size)
{}

void BufferFacts::addFact(Fact *fact)
{
    mutex_lock.lock();
    write_queue->push_back(fact);
    mutex_lock.unlock();
    if (fact->getTimeStamp() > more_recent_timestamp_)
        more_recent_timestamp_ = fact->getTimeStamp();
}

std::vector<Fact *> BufferFacts::getFacts(TimeStamp_t current_time)
{
    mutex_lock.lock();
    auto temp = write_queue;
    write_queue = read_queue;
    read_queue = temp;
    mutex_lock.unlock();

    if (read_queue->empty() == false)
    {
        LOG_INFO << "[BufferFacts] Swapped queues - read_queue has " << read_queue->size() << " facts";
        history_queue.insert(history_queue.end(), read_queue->begin(), read_queue->end());
        read_queue->clear();
        LOG_INFO << "[BufferFacts] Before cleanOldFacts: history_queue size = " << history_queue.size();
        LOG_INFO << "[BufferFacts] current_time = " << current_time << ", ttl_ = " << ttl_;
        for (const auto& fact : history_queue) {
            double age = current_time - fact->getTimeStamp();
            LOG_INFO << "[BufferFacts] Fact " << fact->getId() << " timestamp=" << fact->getTimeStamp()
                     << ", age=" << age << ", will_be_removed=" << (age > ttl_);
        }
        cleanOldFacts(current_time);
        LOG_INFO << "[BufferFacts] After cleanOldFacts: history_queue size = " << history_queue.size();
        std::sort(history_queue.begin(), history_queue.end(), customSort);
    }

    if (history_queue.size() > max_size_)
        history_queue.resize(max_size_);

    return history_queue;
}

void BufferFacts::cleanUsedFacts(const std::set<uint32_t> &ids)
{
    auto new_end = std::remove_if(history_queue.begin(), history_queue.end(),
                                    [ids](Fact *val) { return ids.find(val->getId()) != ids.end(); });
    history_queue.erase(new_end, history_queue.end());

}

void BufferFacts::cleanOldFacts(TimeStamp_t current_time)
{
    auto new_end = std::remove_if(history_queue.begin(), history_queue.end(),
                                    [current_time,this](Fact *val) {
                                        return (current_time - val->getTimeStamp()) > this->ttl_;
                                    });
    history_queue.erase(new_end, history_queue.end());
}

} // namespace procedural