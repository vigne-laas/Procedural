#ifndef PROCEDURAL_TIMESTAMP_H
#define PROCEDURAL_TIMESTAMP_H

#include <tuple>
#include <cmath>
#include <iostream>

namespace procedural {
struct TimeStamp_t
{

    TimeStamp_t(int sec, int nsec) : sec_(sec), nsec_(nsec){}
    TimeStamp_t(uint32_t sec, uint32_t nsec) : sec_(int(sec)), nsec_(int(nsec)){}
    TimeStamp_t() : nsec_(0), sec_(0){}
    explicit TimeStamp_t(double sec)
    {
        sec_ = floor(sec);
        nsec_ = int((sec - sec_) * pow(10, 9));
    }

    int sec_;
    int nsec_;


    friend bool operator<(const TimeStamp_t& left, const TimeStamp_t& right)
    {
        return std::tie(left.sec_, left.nsec_) < std::tie(right.sec_, right.nsec_);
    }
    friend bool operator>(const TimeStamp_t& lhs, const TimeStamp_t& rhs) { return rhs < lhs; }
    friend bool operator<=(const TimeStamp_t& lhs, const TimeStamp_t& rhs) { return !(lhs > rhs); }
    friend bool operator>=(const TimeStamp_t& lhs, const TimeStamp_t& rhs) { return !(lhs < rhs); }
    friend double operator-(const TimeStamp_t& lhs, const TimeStamp_t& rhs)
    { return double((lhs.sec_ - rhs.sec_)) + (lhs.nsec_ - rhs.nsec_) * pow(10, -9); }
    TimeStamp_t removeDeltaT( const TimeStamp_t& rhs) const
    {
        TimeStamp_t res;
        res.sec_ = this->sec_ - rhs.sec_;
        auto temp_nsec = this->nsec_ - rhs.nsec_;
        if (temp_nsec < 0)
        {
            temp_nsec = -temp_nsec;
            res.sec_--;
        }
        res.nsec_ = temp_nsec;

        return res;
    }
    TimeStamp_t addDeltaT(const TimeStamp_t& rhs) const
    {
        TimeStamp_t res;
        res.sec_ = this->sec_ + rhs.sec_;
        auto temp_nsec = this->nsec_ + rhs.nsec_;
        if (temp_nsec > 1 * pow(10, 9))
        {
            temp_nsec = temp_nsec - (int) (1 * pow(10, 9));
            res.sec_++;
        }
        res.nsec_ = temp_nsec;

        return res;
    }
    friend std::ostream& operator<<(std::ostream& os, const TimeStamp_t& lhs)
    {
//        os << lhs.sec_ << "s," << lhs.nsec_ << "ns";
        os << std::to_string(lhs.sec_) << "s," << std::to_string(int(lhs.nsec_/pow(10,9)));
        return os;
    }

    float toFloat() const
    {
        return float(sec_ + (nsec_ * pow(10, -9)));
    }


};

}


#endif //PROCEDURAL_TIMESTAMP_H
