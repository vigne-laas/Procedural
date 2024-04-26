#ifndef PROCEDURAL_VARIABLE_H
#define PROCEDURAL_VARIABLE_H

#include <string>
#include "ontologenius/clients/ontologyClients/ClassClient.h"
namespace procedural {

struct Variable_t {
    explicit Variable_t(const std::string& literal, const std::string& type) : literal_(literal), type_(type),
                                                                               value_(0) {}
    Variable_t(const Variable_t& other) = default;
    Variable_t( Variable_t& other) = default;

    Variable_t(const std::string& literal, const std::string& type, onto::ClassClient* class_manipulator) : literal_(
            literal), type_(type), value_(0)
    {
        this->extend(class_manipulator);
    }

    std::string literal_;
    std::string type_;
    std::unordered_set<std::string> extended_types_;
    uint32_t value_;


    bool isSet() const { return value_ != 0; }

    uint32_t getValue() const { return value_; }

    void setValue(uint32_t value) { value_ = value; }

    void extend(onto::ClassClient* class_manipulator)
    {
        if (extended_types_.empty()) {
            auto res = class_manipulator->getDown(type_);
            for (auto& type: res) {
                extended_types_.insert(type);
            }
        }
    }

    std::string getType() const { return type_; }

    std::string toString() const
    {
        std::string str = toShortString();
        str += "(" + type_ + (extended_types_.empty() ? "" : " : ");
        for (const auto& type: extended_types_) {
            str += type + ", ";
        }
        str += ")";
        return str;
    }

    std::string toShortString() const { return (value_ ? std::to_string(value_) : literal_); }

    friend std::ostream& operator<<(std::ostream& os, const Variable_t& var) { return os << var.toString(); }

    bool operator==(const Variable_t& other) const
    {
        if (type_ != other.type_) {
            std::unordered_set<std::string> intersection;
            std::set_intersection(extended_types_.begin(), extended_types_.end(),
                                  other.extended_types_.begin(), other.extended_types_.end(),
                                  std::inserter(intersection, intersection.begin()));
            if (intersection.empty()) {
                return false;
            }
        }
        if (isSet() and value_ != other.value_)
            return false;
        return true;
        //        return (literal_ == other.literal_) && (value_ == other.value_); }
    }

    bool operator!=(const Variable_t& other) const { return !(*this == other); }
};

} // procedural
#endif //PROCEDURAL_VARIABLE_H
