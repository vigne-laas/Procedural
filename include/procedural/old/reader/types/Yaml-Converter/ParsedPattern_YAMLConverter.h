#ifndef PROCEDURAL_PARSEDPATTERN_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDPATTERN_YAMLCONVERTER_H

#include "procedural/old/reader/types/ParsedPattern.h"

namespace YAML {

template<>
struct convert<procedural::ParsedPattern_t>
{
    static Node encode(const procedural::ParsedPattern_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, procedural::ParsedPattern_t& parsed_pattern)
    {

        if (!node.IsSequence())
        {
            return false;
        }
        int level = 0;
        for (auto iter_node = 0; iter_node != node.size(); iter_node++)
        {

            if (node[iter_node].IsScalar())
            {
                auto key = node[iter_node].as<std::string>();
//                std::cout << "key pattern: " << key << std::endl;
                parsed_pattern.facts.emplace_back(key, level);
            } else
            {
                if (node[iter_node].IsMap()) //TODO or_facts or or_networks
                {
                    for (const auto& map_elmt: node[iter_node])
                    {
//                        std::cout << "key : " << map_elmt.first.as<std::string>() << " value : "
//                                  << map_elmt.second.as<std::string>() << std::endl;
                        auto literal = map_elmt.first.as<std::string>();
                        auto type = map_elmt.second.as<std::string>();
                        parsed_pattern.sub_state_machines.emplace_back(literal, type, level);
                    }

                }
            }
            level++;
        }
        parsed_pattern.max_level = level;
        return true;
    }
};

}

#endif //PROCEDURAL_PARSEDPATTERN_YAMLCONVERTER_H
