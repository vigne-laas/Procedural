#ifndef PROCEDURAL_YAMLREADER_H
#define PROCEDURAL_YAMLREADER_H

#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <experimental/optional>
//#include <yaml.h>
#include <yaml-cpp/yaml.h>
#include <regex>
#include <unordered_set>
#include "procedural/core/Types/Action.h"
#include "procedural/core/Types/PatternTransition.h"
#include "procedural/core/Types/ActionDescription.h"


namespace procedural {

class YamlReader
{
public:
    YamlReader();
    bool read(const std::string& path);
    std::vector<Action*> getActions() { return actions_; };

    std::string actionsCreatedString();

private:
    std::regex pattern_facts_;
    std::regex pattern_description_;
    std::unordered_set<std::string> compose_action_;
    YAML::Node yaml_file_;
    std::vector<Action*> actions_;


    Action* createAction(const std::string& name, const YAML::Node& node);
    Action* createComposeAction(const std::string& name, const YAML::Node& node);

    uint32_t readParameters(const YAML::Node& node);
    std::vector<PatternTransition_t> readFacts(const YAML::Node& node);
    std::vector<ActionDescription_t> readDescription(const YAML::Node& node);

    bool isPrimitiveAction(const YAML::Node& node) { return node["ordered_facts"] && node["description"]; };
    bool isComposeAction(const YAML::Node& node)
    {
        return (static_cast<bool>(node["ordered_facts"]) == false) &&
               (static_cast<bool>(node["description"]) == false) and node.size() >= 1;
    };

    FactPattern* parseFact(const std::string& str_fact);
    procedural::ActionDescription_t parseDescription(const std::string& str_description);

    PatternRecognition createPattern(const std::string& name, const YAML::Node& node, uint32_t ttl = 4);

};

} // namespace procedural

#endif // PROCEDURAL_YAMLREADER_H
