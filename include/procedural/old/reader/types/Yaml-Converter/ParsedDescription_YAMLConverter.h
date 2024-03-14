#ifndef PROCEDURAL_PARSEDDESCRIPTION_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDDESCRIPTION_YAMLCONVERTER_H

#include "procedural/old/reader/types/ParsedDescription.h"

namespace YAML {

template<>
struct convert<procedural::ParsedDescriptions_t>
{
    static Node encode(const procedural::ParsedDescription_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, procedural::ParsedDescriptions_t& description)
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
