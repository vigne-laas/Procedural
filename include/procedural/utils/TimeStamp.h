#ifndef PROCEDURAL_TIMESTAMP_H
#define PROCEDURAL_TIMESTAMP_H

#include <tuple>
#include <math.h>

namespace procedural {
    struct TimeStamp_t {

        TimeStamp_t(int sec, int nsec) : sec_(sec), nsec_(nsec) {}
        TimeStamp_t() : nsec_(0), sec_(0) {}

        int sec_;
        int nsec_;


        friend bool operator<(const TimeStamp_t &left, const TimeStamp_t &right) {
//            if (right.sec_ > left.sec_)
//                return true;
//            if (right.sec_ < left.sec_)
//                return false;
//            else {
//                if (right.nsec_ > left.nsec_)
//                    return true;
//                return false;
//            }
            return std::tie(left.sec_,left.nsec_) < std::tie(right.sec_,right.nsec_);
        }
        friend bool operator>(const TimeStamp_t &lhs, const TimeStamp_t &rhs) { return rhs < lhs; }
        friend bool operator<=(const TimeStamp_t &lhs, const TimeStamp_t &rhs) { return !(lhs > rhs); }
        friend bool operator>=(const TimeStamp_t &lhs, const TimeStamp_t &rhs) { return !(lhs < rhs); }
        friend double operator-(const TimeStamp_t &lhs, const TimeStamp_t &rhs) {return double((lhs.sec_-rhs.sec_))+(lhs.nsec_-rhs.nsec_)*pow(10,-9);}
        friend std::ostream& operator<<(std::ostream& os, const TimeStamp_t &lhs)
        {
            os << lhs.sec_ << "s,"<< lhs.nsec_ <<"ns";
            return os;
        }

        float toFloat(){
            return float(sec_+(nsec_*pow(10,-9)));
        }


    };

}


#endif //PROCEDURAL_TIMESTAMP_H
