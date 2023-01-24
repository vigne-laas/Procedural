#include "test_facts.h"

#include "procedural/core/Types/Fact.h"

#include <iostream>

int main(){
    procedural::Fact::properties_table.add("isA");
    procedural::Fact::properties_table.add("isUnder");
    procedural::Fact F1 = procedural::Fact(false, "plop", "isA", "kyoto_floor");
    std::cout <<"object : " <<F1.getObject() << ":" <<F1.getStringObject() <<std::endl;
    std::cout << F1.toString() << std::endl;
    procedural::Fact::individuals_table.printAll();
    procedural::Fact F2 = procedural::Fact(true, "plop", "is", "kyoto_floor");
    std::cout << F2.toString() << std::endl;


    if(!F2.isValid()){
        std::cout << "F2 is invalid" << std::endl;
    }





    return 0;
}
