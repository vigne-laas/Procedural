#ifndef PROCEDURAL_PARSEDDESCRIPTION_H
#define PROCEDURAL_PARSEDDESCRIPTION_H

#include <regex>

namespace procedural {

struct ParsedDescription_t
{
    ParsedDescription_t() : regex_description(R"(\s*([^\s]*)\s+([^\s]*)\s+([^\s]*)\s*)"), subject(), property(),
                            object()
    {}

    explicit ParsedDescription_t(const std::string& str_value) : regex_description(
            R"(\s*([^\s]*)\s+([^\s]*)\s+([^\s]*)\s*)"), subject(), property(), object()
    {
        parse(str_value);
    };

    void parse(const std::string& str_value)
    {
        std::smatch match;
        std::regex_search(str_value, match, regex_description);
        subject = match[1];
        property = match[2];
        object = match[3];
    }

    friend std::ostream& operator<<(std::ostream& os, const ParsedDescription_t& lhs)
    {
        os << lhs.subject << " " << lhs.property << " " << lhs.object;
        return os;
    }

    std::regex regex_description;
    std::string subject;
    std::string property;
    std::string object;
};


struct ParsedDescriptions_t
{
    ParsedDescriptions_t() : descriptions()
    {};
    bool empty() const {return descriptions.empty();}
    friend std::ostream& operator<<(std::ostream& os, const ParsedDescriptions_t& lhs)
    {
        if(lhs.empty())
            return os;
        for (const auto& description: lhs.descriptions)
            os << description << "\n";
        return os;
    }

    std::vector<ParsedDescription_t> descriptions;
};

}
#endif //PROCEDURAL_PARSEDDESCRIPTION_H
