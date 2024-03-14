#ifndef PROCEDURAL_PARSEDFACTS_YAMLCONVERTER_H
#define PROCEDURAL_PARSEDFACTS_YAMLCONVERTER_H

#include <yaml-cpp/yaml.h>
#include "procedural/old/reader/types/ParsedFacts.h"

namespace YAML {

template<>
struct convert<procedural::ParsedFacts_t>
{
    static Node encode(const procedural::ParsedFacts_t& rhs)
    {
        Node node;
        return node;
    }
    static bool decode(const Node& node, procedural::ParsedFacts_t& parsed_facts)
    {
        if (!node.IsSequence())
        {
            return false;
        }
        uint32_t level = 0;
        for (auto iter_node = 0; iter_node != node.size(); iter_node++)
        {
            if (node[iter_node].IsScalar())
            {
                auto key = node[iter_node].as<std::string>();
//                std::cout << "key facts: " << key << std::endl;
                parsed_facts.facts_.emplace_back(key, level);
            } else
            {
                if (node[iter_node].IsMap())
                {
                    for (auto key_map: node[iter_node])
                    {
//                        std::cout << "key_map: " << key_map.first.as<std::string>() << std::endl;
                        if (key_map.first.as<std::string>() == "parallel")
                        {
                            auto sub_node = node[iter_node]["parallel"];
                            for (auto or_nodes = 0; or_nodes != sub_node.size(); or_nodes++)
                            {
                                if (sub_node[or_nodes].IsScalar())
                                {
                                    auto key = sub_node[or_nodes].as<std::string>();
//                                    std::cout << "key or facts: " << key << std::endl;
                                    parsed_facts.facts_.emplace_back(key, level);
                                }

                            }

                        }

                    }
                }


            }

            level++;

        }
        return true;
    }
};

}

#endif //PROCEDURAL_PARSEDFACTS_YAMLCONVERTER_H
