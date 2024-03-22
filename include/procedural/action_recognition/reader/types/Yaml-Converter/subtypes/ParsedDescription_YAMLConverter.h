#ifndef PROCEDURAL_PARSEDDESCRIPTION_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDDESCRIPTION_YAMLCONVERTER_H

#include <yaml-cpp/yaml.h>
#include "procedural/action_recognition/reader/types/subtypes/ParsedDescription.h"

namespace YAML {

template<>
struct convert<action_recognition::ParsedDescriptions_t>
{
    static Node encode(const action_recognition::ParsedDescription_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, action_recognition::ParsedDescriptions_t& description)
    {
        if (!node.IsSequence())
        {
            return false;
        }
        for (auto iter_node = 0; iter_node != node.size(); iter_node++)
        {
            if (node[iter_node].IsScalar())
            {
                auto key = node[iter_node].as<std::string>();
//                std::cout << "key description : " << key << std::endl;
                description.descriptions.emplace_back(key);
            }

        }


        return true;
    }
};
}// YAML
#endif //PROCEDURAL_PARSEDDESCRIPTION_YAMLCONVERTER_H
