#include "procedural/structures/ObservationFact.h"

namespace procedural {

ObservationFact::ObservationFact(const Fact& fact) : Observation(
        ((int64_t) 0x01 << 63) |
        (int64_t) std::hash<std::string>{}((fact.getAdd() ? "ADD " : "DEL ") + fact.getStrProperty())),
                                                     fact_(fact)
{
    table_variables_.variables[fact.getLiteralObject()] = std::make_shared<Variable_t>(*fact.getObject().second);
    table_variables_.variables[fact.getLiteralSubject()] = std::make_shared<Variable_t>(*fact.getSubject().second);
    fact_.link(table_variables_);

//    LOG_DEBUG << "ObservationFact constructor\n";
//    LOG_DEBUG << fact_.toString() << "\n";
//    LOG_DEBUG << table_variables_.toString() << "\n";
    if (id_ > 0) {
        LOG_ERROR << "\t mask : " << ((int64_t) 0x01 << 63) << "\n";
        LOG_ERROR << "\t hash : "
                  << (int64_t) std::hash<std::string>{}((fact.getAdd() ? "ADD " : "DEL ") + fact.getStrProperty())
                  << "\n";
        throw std::runtime_error("Invalid id");
    }
}


bool ObservationFact::operator==(const Observation& other)
{
//    LOG_INFO << "ObservationFact::operator==\n";
    if (other.getId() > 0) {
//        LOG_INFO << "other.getId() > 0\n";
        return false;
    }
    const auto* other_fact = dynamic_cast<const ObservationFact*>(&other);
    if (other_fact) {
//        LOG_DEBUG << "try to match " << fact_.toString() << " with " << other_fact->fact_.toString() << "\n";
        if (id_ == other.getId()) {
//            LOG_DEBUG << "id_ == other.getId()\n";
//            LOG_DEBUG << "try to match " << fact_.toString() << " with " << other_fact->fact_.toString() << "\n";
            auto res = fact_.match(other_fact->fact_);
            if (res) {
//                LOG_DEBUG << "Matched\n";
                if (!table_variables_.variables[fact_.getLiteralObject()]->isSet())
                    table_variables_.variables[fact_.getLiteralObject()]->value_ = other_fact->fact_.getIdObject();
                if (!table_variables_.variables[fact_.getLiteralSubject()]->isSet())
                    table_variables_.variables[fact_.getStrSubject()]->value_ = other_fact->fact_.getIdSubject();
//                LOG_DEBUG << table_variables_.toString() << "\n";
                return true;
            }

            return false;
        }
//        LOG_DEBUG << "add_ " << fact_.getAdd() << " != other.getAdd() " << other_fact->fact_.getAdd() << "\n";
//        LOG_DEBUG << "property " << fact_.getStrProperty() << " != other.getProperty() "
//                  << other_fact->fact_.getStrProperty() << "\n";
//        LOG_DEBUG << "id_" << id_ << " != other.getId() " << other.getId() << "\n";
//        LOG_DEBUG << "id_ != other.getId()\n";
        return false;
    }
    return false;
}

void ObservationFact::expandProperty(onto::ObjectPropertyClient* object_property_client)
{
    fact_.expandProperty(object_property_client);
}

std::string ObservationFact::toString() const
{
    std::string result = "ObservationFact: ";
    result += fact_.toString();
    return result;

}

void ObservationFact::linkVariables(std::map<std::string, std::shared_ptr<Variable_t>>& variables)
{
//    LOG_DEBUG << "ObservationFact::linkVariables\n";
    Observation::linkVariables(variables);
    fact_.link(table_variables_);
}


} // procedural