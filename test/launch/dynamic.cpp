#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontologenius/API/ontologenius/OntologyManipulator.h"

onto::OntologyManipulator* onto_ptr;

TEST(dynamic_tests, insert)
{
    ros::Rate wait(5);
    std::vector<std::string> res;
    bool res_bool = true;
    std::string test_word = "robot";

    onto_ptr->reasoners.activate("ontologenius::ReasonerGeneralize");

    onto_ptr->feeder.addConcept("human");
    onto_ptr->feeder.addInheritage("man", "human");
    onto_ptr->feeder.addInheritage("woman", "human");
    onto_ptr->feeder.addInheritage("human", "agent");
    onto_ptr->feeder.addInheritage("robot", "agent");
    onto_ptr->feeder.addInheritage("pepper", "robot");
    onto_ptr->feeder.waitUpdate(500);


    res = onto_ptr->classes.getUp(test_word);
    res_bool = ((res.size() == 2) &&
                (find(res.begin(), res.end(), "robot") != res.end()) &&
                (find(res.begin(), res.end(), "agent") != res.end()));
    EXPECT_TRUE(res_bool);

    test_word = "pepper";
    res = onto_ptr->individuals.getUp(test_word);
    res_bool = ((res.size() == 2) &&
                (find(res.begin(), res.end(), "robot") != res.end()) &&
                (find(res.begin(), res.end(), "agent") != res.end()));
    EXPECT_TRUE(res_bool);

    onto_ptr->feeder.addInheritage("alice", "woman");
    onto_ptr->feeder.addInheritage("laura", "woman");
    onto_ptr->feeder.addProperty("laura", "hasLeg", "int", "2");
    onto_ptr->feeder.addProperty("alice", "hasLeg", "int", "2");
    onto_ptr->feeder.waitUpdate(500);
    wait.sleep();

    res = onto_ptr->classes.getOn("woman", "hasLeg");
    res_bool = ((res.size() == 1) &&
                (find(res.begin(), res.end(), "int#2") != res.end()));
    EXPECT_TRUE(res_bool);

    onto_ptr->feeder.removeProperty("alice", "hasLeg", "int", "2");
    onto_ptr->feeder.waitUpdate(500);
    wait.sleep();

    res = onto_ptr->classes.getOn("woman", "hasLeg");
    res_bool = (res.size() == 0);
    EXPECT_TRUE(res_bool);

//    onto_ptr->feeder.addProperty("myself", "", "");
//    onto_ptr->feeder.addProperty("myself", "hasPart", "myself");
//
//    onto_ptr->feeder.addProperty("hasLeg","+", "hasPart");
//    onto_ptr->feeder.waitUpdate(500);
//    wait.sleep();
//
//    res_bool = onto_ptr->objectProperties.exist("hasPart");
//    EXPECT_TRUE(res_bool);
//
//    res = onto_ptr->objectProperties.getUp("hasLeg");
//     res_bool = ((res.size() == 1) &&
//                (find(res.begin(), res.end(), "hasPart") != res.end()));
//    std::cout << "Test getUp hasLeg : " << res_bool << std::endl;
//    for (const auto& word: res) {
//        std::cout << "\t" << word << std::endl;
//    }
//
//    EXPECT_TRUE(res_bool);
//    ROS_DEBUG("Test insert passed");


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ontologenius_dynamic_tester");

    onto::OntologyManipulator onto;
    onto_ptr = &onto;

    ros::Rate wait(1);
    onto.close();
    wait.sleep();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
