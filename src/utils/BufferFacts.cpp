#include <set>
#include "procedural/utils/BufferFacts.h"
#include <algorithm>
#include <iostream>

namespace procedural {

    BufferFacts::BufferFacts(double ttl, int max_size)
            : primary_queue(), secondary_queue(), history_queue(), read_queue(&primary_queue),
              write_queue(&secondary_queue), ttl_(ttl), max_size_(max_size),more_recent_timestamp_() {
    }


    void BufferFacts::addFact(Fact *fact) {
        mutex_lock.lock();
        write_queue->push_back(fact);
        mutex_lock.unlock();
        if (fact->getTimeStamp() > more_recent_timestamp_)
        {
            more_recent_timestamp_ = fact->getTimeStamp();
            std::cout << "Buffer more recent timestamp : " << more_recent_timestamp_<< std::endl;
        }

    }


    std::vector<Fact *> BufferFacts::getFacts(TimeStamp_t current_time) {
        mutex_lock.lock();
        auto temp = write_queue;
        write_queue = read_queue;
        read_queue = temp;
        mutex_lock.unlock();
        if (read_queue->empty() == false) {
            history_queue.insert(history_queue.end(), read_queue->begin(), read_queue->end());
            read_queue->clear();
            cleanOldFacts(current_time);
            std::sort(history_queue.begin(), history_queue.end(), customSort);
        }
        if (history_queue.size() > max_size_)
            history_queue.resize(max_size_);
        return history_queue;
    }

    void BufferFacts::cleanUsedFacts(const std::set<uint32_t> &ids) {

        auto new_end = std::remove_if(history_queue.begin(), history_queue.end(),
                                      [ids](Fact *val) { return ids.find(val->getId()) != ids.end(); });
        history_queue.erase(new_end, history_queue.end());

    }

    void BufferFacts::cleanOldFacts(TimeStamp_t current_time) {
        auto new_end = std::remove_if(history_queue.begin(), history_queue.end(),
                                      [current_time,this](Fact *val) {
                                          return (current_time - val->getTimeStamp()) > this->ttl_;
                                      });
        history_queue.erase(new_end, history_queue.end());


    }


} // procedural