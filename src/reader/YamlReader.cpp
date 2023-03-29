#include "procedural/reader/YamlReader.h"


#include <fstream>
#include <iostream>
#include <regex>
#include <filesystem>

namespace procedural {

YamlReader::YamlReader() : pattern_facts_(R"(\s*(NOT)?\s*\?([^\s]*)\s+([^\s]*)\s+\?([^\s]*)\s*(REQUIRED)?)"),
                           pattern_description_(R"(\s*([^\s]*)\s+([^\s]*)\s+([^\s]*)\s*)")
{

}


bool YamlReader::read(const std::string& path)
{
    if (std::filesystem::exists(path))
    {
        yaml_file_ = YAML::LoadFile(path);
        for (YAML::const_iterator it = yaml_file_.begin(); it != yaml_file_.end(); ++it)
        {
            std::cout << "Action find " << it->first.as<std::string>() << std::endl;
            if (isPrimitiveAction(it->second))
                actions_.push_back(createAction(it->first.as<std::string>(), it->second));
            if (isComposeAction(it->second))
                actions_.push_back(createComposeAction(it->first.as<std::string>(), it->second));
        }
    } else
        return false;
    return true;
}

std::string YamlReader::actionsCreatedString()
{
    std::string res;
    for (auto& action: actions_)
        res += action->toString();
    return res;
}

// Private Part

Action* YamlReader::createAction(const std::string& name, const YAML::Node& node)
{

    PatternRecognition pattern = createPattern(name, node);
    Action* action = new Action(name);
    action->addPatterns(pattern);
    return action;

}

Action* YamlReader::createComposeAction(const std::string& name, const YAML::Node& node)
{
    uint32_t ttl = 4;
    std::vector<PatternRecognition> vect_pat;

    for (auto it: node)
    {
        std::string key = it.first.as<std::string>();
        if (key == "parameters")
            ttl = readParameters(it.second);
        else
            vect_pat.push_back(createPattern(name + "_" + key, it.second, ttl));
    }

    Action* action = new Action(name);
    for (auto& pat: vect_pat)
        action->addPatterns(pat);
    return action;
}

uint32_t YamlReader::readParameters(const YAML::Node& node)
{
//    std::cout << node["ttl"] << std::endl;
    if (node["ttl"])
        return 10; // FIXME issue to fix bad conversion of node["ttl"].as<uint32_t>
    else
        return 4;

}
std::vector<PatternTransition_t> YamlReader::readFacts(const YAML::Node& node)
{
    std::vector<PatternTransition_t> net;
    uint32_t current_state_id = 0;
    uint32_t last_required = 0;
    for (auto it = 0; it != node.size(); it++)
    {
        if (node[it].IsScalar())
        {
            FactPattern* fact_pattern = parseFact(
                    node[it].as<std::string>()); // Pas terrible voir pour faire un fonction mais un peu trop de parametres ... net*,fact_pattern*,last_required*,current_id*
            for (auto index = last_required; index <= current_state_id; index++)
                net.emplace_back(index, fact_pattern, current_state_id + 1);
            if (fact_pattern->isRequired())
                last_required = current_state_id + 1;

        } else
        {
            if (node[it]["or_facts"])
            {
                YAML::Node subnode = node[it]["or_facts"];
                FactPattern* fact_pattern;
                for (auto indexSub = 0; indexSub != subnode.size(); indexSub++)
                {
                    fact_pattern = parseFact(subnode[indexSub].as<std::string>());
                    for (auto index = last_required; index <= current_state_id; index++)
                        net.emplace_back(index, fact_pattern, current_state_id + 1);
                    if (fact_pattern->isRequired())
                        last_required = current_state_id + 1;
                }
            }

        }
        current_state_id++;
    }
//    for (auto& trans: net)
//        std::cout << trans.origin_state << "==" << trans.fact->toString() << "==>" << trans.next_state << std::endl;
    return net;
}
std::vector<ActionDescription_t> YamlReader::readDescription(const YAML::Node& node)
{
    std::vector<ActionDescription_t> descriptions;
    for (auto id = 0; id != node.size(); id++)
    {
        descriptions.push_back(parseDescription(node[id].as<std::string>()));
    }
    return descriptions;
}


FactPattern* YamlReader::parseFact(const std::string& str_fact)
{
    std::smatch results;
    std::regex_search(str_fact, results, pattern_facts_);
    auto* fact = new FactPattern(results[1] != "NOT", results[2], results[3], results[4], results[5] == "REQUIRED");
    return fact;
}


procedural::ActionDescription_t YamlReader::parseDescription(const std::string& str_description)
{
    std::smatch match;
    std::regex_search(str_description, match, pattern_description_);
//    for(auto& sub_value :match)
//        std::cout << sub_value << std::endl;
    return {match[1], match[2], match[3]};
}

PatternRecognition YamlReader::createPattern(const std::string& name, const YAML::Node& node, uint32_t ttl)
{
    std::vector<PatternTransition_t> net;
    std::vector<ActionDescription_t> descriptions;
    for (auto iter_node: node)
    {
        std::string key = iter_node.first.as<std::string>();
        if (key == "parameters")
            ttl = readParameters(iter_node.second);
        if (key == "ordered_facts")
            net = readFacts(iter_node.second);
        if (key == "description")
            descriptions = readDescription(iter_node.second);

    }
    PatternRecognition pattern = PatternRecognition(name, net, descriptions, ttl);
    return pattern;
}


} // namespace procedural