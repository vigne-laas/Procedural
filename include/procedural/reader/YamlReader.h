#ifndef PROCEDURAL_YAMLREADER_H
#define PROCEDURAL_YAMLREADER_H

#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <experimental/optional>
//#include <yaml.h>
#include <yaml-cpp/yaml.h>
#include <regex>


namespace procedural {

class YamlReader;

class Action
{
public:
    Action(const YAML::Node& yamlAction, const std::string name);

    const std::vector<std::string>& getFacts();

    const std::vector<std::string>& getDescriptions();

    const std::vector<std::string>& getVariables();

private:
    std::string name_;
    std::regex patternDescription;
    std::regex patternFacts;

    void readDescriptions(const YAML::Node& yamlDescriptions);
    void readFacts(const YAML::Node& yamlFacts);
    void parseFact(const std::string& fact);

    bool ordered_ = false;
    std::vector<std::string> variables_;
    std::vector<std::string> facts_;
    std::vector<std::string> descriptions_;

};

//class YamlElement
//{
//  friend YamlReader;
//public:
//  YamlElement operator[](const std::string& name)
//  {
//    if(subelem)
//    {
//      if(subelem.value().find(name) != subelem.value().end())
//        return subelem.value()[name];
//      else
//        return YamlElement();
//    }
//    else
//      return YamlElement();
//  }
//
//  std::vector<std::string> value()
//  {
//    if(data)
//      return data.value();
//    else
//      return {};
//  }
//
//  std::vector<std::string> getElementsKeys()
//  {
//    std::vector<std::string> res;
//    if(subelem)
//      std::transform(subelem.value().cbegin(), subelem.value().cend(),
//                     std::back_inserter(res),
//                     [](const std::pair<std::string, YamlElement>& elem){ return elem.first;});
//    return res;
//  }
//
//  bool keyExists(const std::string& key)
//  {
//    if(subelem)
//      return (subelem.value().find(key) != subelem.value().end());
//    else
//      return false;
//  }
//
//private:
//  std::experimental::optional<std::vector<std::string>> data;
//  std::experimental::optional<std::map<std::string, YamlElement>> subelem;
//};

class YamlReader
{
public:
    bool read(const std::string& path);

    void display();

//    YamlElement operator[](const std::string& name)
//    {
//        if (elements_.find(name) != elements_.end())
//            return elements_[name];
//        else
//            return YamlElement();
//    }
//
//    std::vector<std::string> getKeys()
//    {
//        std::vector<std::string> res;
//        std::transform(elements_.cbegin(), elements_.cend(),
//                       std::back_inserter(res),
//                       [](const std::pair<std::string, YamlElement>& elem) { return elem.first; });
//        return res;
//    }

private:
//    std::map<std::string, YamlElement> elements_;
//
//    std::map<std::string, YamlElement> read(const std::vector<std::string>& lines, size_t& current_line);
//
//    YamlElement readList(const std::vector<std::string>& lines, size_t& current_line);
//
//    void display(const std::map<std::string, YamlElement>& config, size_t nb = 0);
//
//    void displayElement(const std::pair<std::string, YamlElement>& it, size_t nb);
//
//    void displayTab(size_t nb);
//
//    void removeComment(std::string& line);
//
//    size_t countSpaces(const std::string& line);
};

} // namespace procedural

#endif // PROCEDURAL_YAMLREADER_H
