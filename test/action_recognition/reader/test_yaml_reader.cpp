#include <gtest/gtest.h>
#include <ros/ros.h>
#include "procedural/action_recognition/reader/YamlReader.h"
#include <ros/package.h>

namespace action_recognition {

class YamlReaderTest : public ::testing::Test {
protected:
    YamlReader yamlReader;
};



TEST_F(YamlReaderTest, TestRead) {
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
    ASSERT_TRUE(yamlReader.read(file_path));
}

TEST_F(YamlReaderTest, TestGetSimpleActions) {
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
    yamlReader.read(file_path);
    auto simpleActions = yamlReader.getSimpleActions();

    // Add assertions based on your expected simple actions
    ASSERT_EQ(simpleActions.size(), 2); // Assuming you expect 2 simple actions
    ASSERT_EQ(simpleActions[0].getName(), "Release"); // Replace with your expected action name
    ASSERT_EQ(simpleActions[1].getName(), "Grasp"); // Replace with your expected action name
}

TEST_F(YamlReaderTest, TestSimpleActionsDescription) {
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
    yamlReader.read(file_path);
    auto simpleActions = yamlReader.getSimpleActions();

    // Add assertions based on your expected simple actions
    ASSERT_EQ(simpleActions.size(), 2); // Assuming you expect 2 simple actions
    ASSERT_EQ(simpleActions[0].getName(), "Release"); // Replace with your expected action name
    ASSERT_EQ(simpleActions[0].descriptions.size(), 4); // Assuming you expect 4 description
    auto description = simpleActions[0].descriptions.descriptions;
    ASSERT_EQ(description[0].subject, "??");
    ASSERT_EQ(description[0].property, "isA");
    ASSERT_EQ(description[0].object, "ReleaseAction");


}

TEST_F(YamlReaderTest, TestParameters) {
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
    yamlReader.read(file_path);
    auto simpleActions = yamlReader.getSimpleActions();

    // Add assertions based on your expected parameters
    ASSERT_EQ(simpleActions[0].parameters.ttl, 20); // Replace with your expected ttl value
}

TEST_F(YamlReaderTest, TestSequence) {
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
    yamlReader.read(file_path);
    auto simpleActions = yamlReader.getSimpleActions();

    // Verify the sequence
    ASSERT_EQ(simpleActions[0].facts.facts_.size(), 2); // Assuming you expect 2 sequence items
    ASSERT_EQ(simpleActions[0].facts.facts_[0].subject, "A"); // Replace with your expected sequence item
    ASSERT_EQ(simpleActions[0].facts.facts_[0].property, "hasHandMovingToward"); // Replace with your expected sequence item
    ASSERT_EQ(simpleActions[0].facts.facts_[0].object, "S"); // Replace with your expected sequence item
}



TEST_F(YamlReaderTest, TestGetComposedActions) {
    std::string package_path = ros::package::getPath("procedural");
    std::string file_path = package_path + "/test/action_recognition/reader/test_action.yaml";
    yamlReader.read(file_path);
    auto composedActions = yamlReader.getComposedActions();

    // Add assertions based on your expected composed actions
    ASSERT_EQ(composedActions.size(), 1); // Assuming you expect 1 composed action
    ASSERT_EQ(composedActions[0].getName(), "Pick_In"); // Replace with your expected action name
}

}  // namespace action_recognition

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "yaml_reader_test");
    return RUN_ALL_TESTS();
}