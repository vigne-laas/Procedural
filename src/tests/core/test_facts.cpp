#include "procedural/core/Types/Fact.h"

#include <iostream>

int main()
{
    procedural::Fact::properties_table.add("isA");
    procedural::Fact::properties_table.add("isUnder");

    procedural::Fact fact_1 = procedural::Fact(false, "plop", "isA", "kyoto_floor",1);
    std::cout << "fact_1 = " << fact_1.toString() << std::endl;

    if(fact_1.isValid())
        std::cout << "fact_1 is VALID" << std::endl;
    else
        std::cout << "fact_1 is INVALID" << std::endl;

    procedural::Fact fact_2 = procedural::Fact(true, "plop", "is", "kyoto_floor",2);
    std::cout << "fact_2 = " << fact_2.toString() << std::endl;

    if(fact_2.isValid())
        std::cout << "fact_2 is VALID" << std::endl;
    else
        std::cout << "fact_2 is INVALID" << std::endl;

    std::cout << procedural::Fact::individuals_table.toString() << std::endl;

    return 0;
}
