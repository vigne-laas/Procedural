cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(procedural)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )

find_package(yaml-cpp REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES procedural_lib
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${YAML_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
        src/core/Types/PatternFact.cpp
        src/core/Types/WordTable.cpp
        src/core/Types/Fact.cpp
        )

add_library(${PROJECT_NAME}_network_lib
        src/core/Graph/TransitionFact.cpp
        src/core/Graph/State.cpp
        src/core/Graph/Network.cpp
        src/core/Types/Action.cpp
        src/core/Types/PatternRecognition.cpp
        src/core/Graph/TransitionNetwork.cpp)
target_link_libraries(${PROJECT_NAME}_network_lib ${PROJECT_NAME}_lib)

add_library(${PROJECT_NAME}_reader_lib
        src/reader/YamlReader.cpp
        )
target_link_libraries(${PROJECT_NAME}_reader_lib yaml-cpp)

add_library(${PROJECT_NAME}_feed_lib
        src/feeder/Feeder.cpp
        )

##################################
##      Test executables        ##
##################################

add_executable(test_facts src/tests/core/test_facts.cpp)
add_dependencies(test_facts ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_facts
        ${PROJECT_NAME}_lib
        )

add_executable(test_wordtable src/tests/core/test_wordtable.cpp)
add_dependencies(test_wordtable ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_wordtable
        ${PROJECT_NAME}_lib
        )

add_executable(test_network src/tests/graph/test_network.cpp)
add_dependencies(test_network ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_network
        ${PROJECT_NAME}_lib
        ${PROJECT_NAME}_network_lib
        )

add_executable(test_reader src/tests/reader/test_reader.cpp)
add_dependencies(test_reader ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_reader
        ${PROJECT_NAME}_reader_lib
        yaml-cpp
        )

#add_executable(test_action src/tests/test_action.cpp)
#add_dependencies(test_action ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_action
#       ${PROJECT_NAME}_lib
#       ${PROJECT_NAME}_network_lib
#       ${catkin_LIBRARIES}
#       )
add_executable(test_action2 src/tests/test_action2.cpp)
add_dependencies(test_action2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_action2
        ${PROJECT_NAME}_lib
        ${PROJECT_NAME}_network_lib
        ${catkin_LIBRARIES}
        )

#add_executable(test_feeder src/tests/feeder/test_feeder.cpp)
#add_dependencies(test_feeder ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_feeder
#        ${PROJECT_NAME}_feed_lib
#        )
#add_executable(test_feeder src/tests/test_feeder.cpp)
#add_dependencies(test_feeder ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_feeder
#        ${PROJECT_NAME}_lib
#        ${PROJECT_NAME}_feed_lib
#        )

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
