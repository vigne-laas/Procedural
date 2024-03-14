#include "procedural/old/core/Types/WordTable.h"
#include <iostream>

int main()
{
    procedural::WordTable Table = procedural::WordTable();

    Table.add("coucou");

    std::unordered_set<std::string> list;
    list.insert("test");
    list.insert("ajout");
    list.insert("depuis");
    list.insert("une");
    list.insert("liste");
    list.insert("ajout");
    Table.add(list);

    Table.add("coucou");

    std::cout << Table.toString() << std::endl;

    std::cout << "bonjour = " << Table["bonjour"] << std::endl;
    std::cout << "coucou = " << Table["coucou"] << std::endl;

    std::cout << "0 = " << Table[0] << std::endl;
    std::cout << "1 = " << Table[1] << std::endl;

    std::cout << Table.toString() << std::endl;

    std::cout << "get plop = " << Table.get("plop") << std::endl;
    std::cout << "getConst plip = " << Table.getConst("plip") << std::endl;

    return 0;
}
