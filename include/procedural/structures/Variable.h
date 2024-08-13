#ifndef PROCEDURAL_VARIABLE_H
#define PROCEDURAL_VARIABLE_H

#include <string>
#include "ontologenius/clients/ontologyClients/ClassClient.h"
#include "procedural/utils/Logger.h"
#include "procedural/utils/WordTable.h"

namespace procedural {

struct Variable_t {
    explicit Variable_t(const std::string& type) : type_(type), value_(0) {}

    Variable_t(const Variable_t& other) = default;

    Variable_t(Variable_t& other) = default;

    Variable_t(const std::string& type, onto::ClassClient* class_manipulator) : type_(type),
                                                                                value_(0)
    {
        this->extend(class_manipulator);
    }

//    void remapTo(std::shared_ptr<Variable_t> remap_var)
//    {
//        remaps_ = remap_var;
//    }

//    bool isRemap() const
//    {
//        return remaps_ != nullptr;
//    }


//    std::string literal_;
    std::string type_;
    std::unordered_set<std::string> extended_types_;
    uint32_t value_;
//    std::shared_ptr<Variable_t> remaps_;


    bool isSet() const { return value_ != 0; }

    uint32_t getValue() const { return value_; }


    void setValue(uint32_t value)
    {
        LOG_DEBUG << "new value : " << value << "\n";
        value_ = value;
    }

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

    void setType(const std::string& type) { type_ = type; }


    std::unordered_set<std::string> getExtendedTypes() const { return extended_types_; }

    std::string toString(const std::string& literal = "") const
    {

        std::string str = toShortString(literal);
        str += "(" + type_ + (extended_types_.empty() ? "" : " : ");
        for (const auto& type: extended_types_) {
            str += type + ", ";
        }
        str += ")";

        return str;
    }

    std::string toShortString(const std::string& literal = "unset") const
    {
        return (getValue() ? WordTable::individuals_table[getValue()] + ":" + std::to_string(getValue()) : literal);
    }

    friend std::ostream& operator<<(std::ostream& os, const Variable_t& var) { return os << var.toString(); }

    bool operator==(const Variable_t& other) const
    {
//        LOG_DEBUG << "Compare Variable_t\n";
//        LOG_DEBUG << toString() << " =? " << other.toString() << "\n";
        if (type_ != other.type_) {
            if (other.type_.empty() or type_.empty()) {
                LOG_ERROR << "type_ or other.type_ empty\n";
                return false;
            }
//            LOG_DEBUG << "type_ != other.type_\n";
            std::unordered_set<std::string> intersection;
            std::set_intersection(extended_types_.begin(), extended_types_.end(),
                                  other.extended_types_.begin(), other.extended_types_.end(),
                                  std::inserter(intersection, intersection.begin()));
            for (const auto& type: intersection) {
//                LOG_DEBUG << "intersection : " << type << "\n";
            }
            if (intersection.empty()) {
//                LOG_DEBUG << "intersection empty\n";
                return false;
            }
        }
//        LOG_DEBUG << "type_ == other.type_\n";
        if (isSet())
        {
//            LOG_DEBUG << "value : " << value_ << " =? " << other.value_ << "\n";
            return value_ == other.value_;
        }
        return true;
    }


    bool match(const Variable_t& other)
    {
        if(*this == other)
        {

            if(!isSet())
                value_ = other.value_;
            return true;
        }
        return false;
    }

    bool operator!=(const Variable_t& other) const { return !(*this == other); }
};

} // procedural
#endif //PROCEDURAL_VARIABLE_H
