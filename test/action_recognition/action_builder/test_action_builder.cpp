#include <gtest/gtest.h>
#include <ros/package.h>
#include "procedural/action_recognition/reader/YamlReader.h"
#include "procedural/action_recognition/builder/ActionBuilder.h"

namespace procedural {

class ActionBuilderTest : public ::testing::Test {
protected:
    YamlReader yamlReader;
    std::vector<ParsedSimpleAction_t> simpleActions;
    std::vector<ParsedComposedAction_t> composedActions;

    void SetUp() override
    {
        std::string package_path = ros::package::getPath("procedural");
        std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
        yamlReader.read(file_path);
        simpleActions = yamlReader.getSimpleActions();
        composedActions = yamlReader.getComposedActions();
    }
};

//TEST_F(ActionBuilderTest, TestBuildSimpleActions)
//{
//    composedActions.clear();
//    ActionBuilder actionBuilder(simpleActions, composedActions);
//    auto actions = actionBuilder.getActions();
//    ASSERT_EQ(actions.size(), 2);  // Assuming you expect 2 simple actions
//    ASSERT_EQ(actions[0]->getName(), "Release");  // Replace with your expected action name
//    ASSERT_EQ(actions[1]->getName(), "Grasp");  // Replace with your expected action name
//    auto action = actions[0];
//    auto graph = action->getFactory();
//    ASSERT_EQ(graph->getState(), GraphState::Closed);
//
//    // Add assertions to check the built actions
//}

TEST_F(ActionBuilderTest, TestBuildComposedActions)
{
    ActionBuilder actionBuilder(simpleActions, composedActions,"/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/dot/debug/builder/");
    auto actions = actionBuilder.getActions();
    ASSERT_EQ(actions.size(), 3);  // Assuming you expect 2 composed actions
    ASSERT_EQ(actions[0]->getName(), "Release");  // Replace with your expected action name
    ASSERT_EQ(actions[1]->getName(), "Grasp");  // Replace with your expected action name
    ASSERT_EQ(actions[2]->getName(), "Pick_In");  // Replace with your expected action name
    auto action = actions[0];
    auto graph = action->getFactory();
    ASSERT_EQ(graph->getState(), GraphState::Closed);
    // Add assertions to check the built actions
}
//
//TEST_F(ActionBuilderTest, TestBuildComposedActionswithoutSimpleActions)
//{
//    simpleActions.clear();
//    try {
//        ActionBuilder actionBuilder(simpleActions, composedActions);
//        FAIL() << "Expected ActionBuilderException";
//    } catch (const ActionBuilderException& e) {
//        ASSERT_STREQ(e.what(), "Failed to build actions");
//    }
//
//}

}  // namespace procedural

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "action_builder_test");
    return RUN_ALL_TESTS();
}