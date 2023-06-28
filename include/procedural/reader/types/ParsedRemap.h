#ifndef PROCEDURAL_PARSEDREMAP_H
#define PROCEDURAL_PARSEDREMAP_H
namespace procedural {
struct ParsedRemap_t
{
    ParsedRemap_t()=default;

    void addValue(const std::string& origine, const std::string& destination)
    {
        remap.insert(std::make_pair(origine, destination));
    }
    friend std::ostream& operator<<(std::ostream& os, const ParsedRemap_t& lhs)
    {
        for (const auto& map_elmt: lhs.remap)
            os << map_elmt.first << " => " << map_elmt.second << "\n";
        return os;
    }

    std::map<std::string, std::string> remap;


};
struct ParsedRemaps_t
{
    ParsedRemaps_t() : regex_origine(R"(\s*([^\s]*)\.([^\s]*)\s*)"), remaps()
    {};
    bool empty() const
    { return remaps.empty(); }

    void addRemap(const std::string& origine, const std::string& destination)
    {
        std::smatch results;
        std::regex_search(origine, results, regex_origine);
//        for(auto& match : results)
//            std::cout << "match : " << match <<std::endl;
        auto value = remaps.find(results[1]);
        if (value != remaps.end())
            value->second.addValue(results[2], destination);
        else
        {
            ParsedRemap_t remap_local;
            remap_local.addValue(results[2], destination);
            remaps.insert(std::make_pair(results[1], remap_local));
        }

    }

    friend std::ostream& operator<<(std::ostream& os, const ParsedRemaps_t& lhs)
    {
        for (const auto& map_elmt: lhs.remaps)
            os << "State Machine literal " << map_elmt.first << " : \n" << map_elmt.second << "\n";
        return os;
    }
    std::regex regex_origine;
    std::map<std::string, ParsedRemap_t> remaps;

};
}

#endif //PROCEDURAL_PARSEDREMAP_H
