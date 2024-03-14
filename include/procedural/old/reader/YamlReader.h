#ifndef PROCEDURAL_YAMLREADER_H
#define PROCEDURAL_YAMLREADER_H

#include <regex>
#include <yaml-cpp/yaml.h>
#include "procedural/old/reader/types/ParsedSimpleAction.h"
#include "procedural/old/reader/types/ParsedComposedAction.h"

namespace procedural {

class YamlReader
{
public:
    YamlReader() {}
    
    bool read(const std::string& path);

    std::vector<ParsedSimpleAction_t> getSimpleActions() { return simple_actions_; };
    std::vector<ParsedComposedAction_t>& getComposedActions() { return composed_actions_; };

private :
    bool parse();

    std::regex pattern_facts_;
    std::regex pattern_description_;
    YAML::Node yaml_file_;

    std::vector<ParsedSimpleAction_t> simple_actions_;
    std::vector<ParsedComposedAction_t> composed_actions_;

    bool isSimpleAction(const YAML::Node& node);
    bool isComposedAction(const YAML::Node& node);
};

} // namespace procedural

#endif //PROCEDURAL_YAMLREADER_H
