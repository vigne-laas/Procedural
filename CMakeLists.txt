cmake_minimum_required(VERSION 3.7)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(procedural)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        ontologenius
        mementar
)

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_definitions(-DANTLR4CPP_SHARED)
# compiler must be 17
SET(CMAKE_CXX_STANDARD 17)
# using /MD flag for antlr4_runtime (for Visual C++ compilers only)
set(ANTLR4_WITH_STATIC_CRT OFF)
#add_compile_options(-fPIC)
# Specify the version of the antlr4 library needed for this project.
# By default the latest version of antlr4 will be used.  You can specify a
# specific, stable version by setting a repository tag value or a link
# to a zip file containing the libary source.
set(ANTLR4_TAG 4.13.0)
# add external build for antlrcpp
include(ExternalAntlr4Cpp)
# add antrl4cpp artifacts to project environment
include_directories(${ANTLR4_INCLUDE_DIRS})
# set variable pointing to the antlr tool that supports C++
# this is not required if the jar file can be found under PATH environment
set(ANTLR_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/tools/antlr-4.13.0-complete.jar)
# add macros to generate ANTLR Cpp code from grammar
find_package(ANTLR REQUIRED)

# Call macro to add lexer and grammar to your build dependencies.
antlr_target(SampleGrammarLexer ${CMAKE_CURRENT_SOURCE_DIR}/include/procedural/reader/grammar/HATPLexer.g4 LEXER
        PACKAGE procedural)
antlr_target(SampleGrammarParser ${CMAKE_CURRENT_SOURCE_DIR}/include/procedural/reader/grammar/HATPParser.g4 PARSER VISITOR LISTENER
        PACKAGE procedural
        DEPENDS_ANTLR SampleGrammarLexer
        COMPILE_FLAGS -lib ${ANTLR_SampleGrammarLexer_OUTPUT_DIR})

message("output : ${ANTLR_SampleGrammarLexer_OUTPUT_DIR}")
include_directories(${ANTLR_SampleGrammarLexer_OUTPUT_DIR})
include_directories(${ANTLR_SampleGrammarParser_OUTPUT_DIR})
#include_directories(${ANTLR_HATPGrammar_OUTPUT_DIR})
# Call macro to add lexer and grammar to your build dependencies.
# NOTE: Here, we define "antlrcpptest" as our project's namespace
#antlr4cpp_process_grammar(demo procedural ${CMAKE_CURRENT_SOURCE_DIR}/include/procedural/reader/grammar/htn.g4)
# include generated files in project environment
#include_directories(${antlr4cpp_include_dirs_procedural})

find_package(yaml-cpp REQUIRED)
find_package(ontologenius 0.3.0 REQUIRED)

#find_package(ANTLR REQUIRED)
#antlr_target(GrammarParser  ${CMAKE_CURRENT_SOURCE_DIR}/include/procedural/reader/grammar/htn.g4 VISITOR OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/procedural/reader/HATPParser)

#find_package(ANTLR REQUIRED)
#antlr_target(GrammarParser  ${CMAKE_CURRENT_SOURCE_DIR}/include/procedural/reader/grammar/htn.g4 VISITOR OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/procedural/reader/HATPParser)

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
        CATKIN_DEPENDS roscpp rospy std_msgs ontologenius mementar
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${YAML_INCLUDE_DIRS}
        ${ontologenius_INCLUDE_DIRS}
        ${MEMENTAR_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_types_lib
        src/core/Types/PatternFact.cpp
        src/core/Types/WordTable.cpp
        src/core/Types/Fact.cpp
)

add_library(${PROJECT_NAME}_graph_lib
        src/core/Types/ActionMethod.cpp # TODO pb de lien si dans types_lib
        src/core/Types/Action.cpp # TODO pb de lien si dans types_lib
        src/core/Graph/Transitions/TransitionFact.cpp
        src/core/Graph/State.cpp
        src/core/Graph/StateMachine.cpp
        src/core/Graph/Transitions/TransitionAction.cpp)
target_link_libraries(${PROJECT_NAME}_graph_lib ${PROJECT_NAME}_types_lib ${ontologenius_LIBRARIES})

add_library(${PROJECT_NAME}_reader_lib
        src/reader/YamlReader.cpp)
target_link_libraries(${PROJECT_NAME}_reader_lib
        yaml-cpp
)

add_library(${PROJECT_NAME}_domain_reader_lib
        src/reader/DomainReader.cpp ${ANTLR_SampleGrammarLexer_CXX_OUTPUTS}
        ${ANTLR_SampleGrammarParser_CXX_OUTPUTS} src/reader/HATPListener.cpp)
target_link_libraries(${PROJECT_NAME}_domain_reader_lib
        antlr4_shared
)
add_custom_command(TARGET ${PROJECT_NAME}_domain_reader_lib
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
        -E copy ${ANTLR4_RUNTIME_LIBRARIES} .
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME}_htn_type src/core/Types/Task.cpp src/core/Types/Method.cpp)
add_library(${PROJECT_NAME}_htn_graph src/core/Graph/Transitions/TransitionActionMethod.cpp
        src/core/Graph/Transitions/TransitionTask.cpp
)

add_library(${PROJECT_NAME}_builder_lib
        src/builder/ActionBuilder.cpp
        src/builder/HTNBuilder.cpp)

target_link_libraries(${PROJECT_NAME}_builder_lib
        ${PROJECT_NAME}_graph_lib
        ${PROJECT_NAME}_types_lib
        ${PROJECT_NAME}_domain_reader_lib
        ${PROJECT_NAME}_htn_type
        ${PROJECT_NAME}_htn_graph
)

add_library(${PROJECT_NAME}_utils_lib
        src/utils/BufferFacts.cpp
        src/core/ActionRecognition.cpp
)
target_link_libraries(${PROJECT_NAME}_utils_lib ${PROJECT_NAME}_graph_lib)

add_library(${PROJECT_NAME}_feed_lib
        src/feeder/Feeder.cpp)
target_link_libraries(${PROJECT_NAME}_feed_lib ${PROJECT_NAME}_utils_lib)

add_library(${PROJECT_NAME}_interface_lib
        src/RosInterface.cpp)
target_link_libraries(${PROJECT_NAME}_interface_lib ${PROJECT_NAME}_feed_lib
        ${PROJECT_NAME}_reader_lib
        ${PROJECT_NAME}_builder_lib
        ${PROJECT_NAME}_utils_lib
        ${catkin_LIBRARIES})

#add_library(${PROJECT_NAME}_domaine_parser_lib
#        include/procedural/reader/)
#target_link_libraries(${PROJECT_NAME}_interface_lib ${PROJECT_NAME}_feed_lib
#        ${PROJECT_NAME}_reader_lib
#        ${PROJECT_NAME}_builder_lib
#        ${PROJECT_NAME}_utils_lib
#        ${catkin_LIBRARIES})

##################################
##      Test executables        ##
##################################


catkin_install_python(PROGRAMS scripts/testPublisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#add_executable(test_facts src/tests/core/test_facts.cpp)
#add_dependencies(test_facts ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_facts
#        ${PROJECT_NAME}_types_lib
#        )
#
#add_executable(test_wordtable src/tests/core/test_wordtable.cpp)
#add_dependencies(test_wordtable ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_wordtable
#        ${PROJECT_NAME}_types_lib
#        )
#
#add_executable(test_network src/tests/graph/test_network.cpp)
#add_dependencies(test_network ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_network
#        ${PROJECT_NAME}_types_lib
#        ${PROJECT_NAME}_graph_lib
#        )

add_executable(test_reader src/tests/reader/test_reader.cpp)
add_dependencies(test_reader ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_reader
        ${PROJECT_NAME}_reader_lib
        ${PROJECT_NAME}_graph_lib
        ${PROJECT_NAME}_builder_lib
        ${PROJECT_NAME}_utils_lib
        ${catkin_LIBRARIES}
)


add_executable(test_domain_reader src/tests/reader/test_domain_builder.cpp)
add_dependencies(test_domain_reader ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_domain_reader
        ${PROJECT_NAME}_reader_lib
        ${PROJECT_NAME}_builder_lib
        ${catkin_LIBRARIES}
)
#add_executable(test_action src/tests/test_action.cpp)
#add_dependencies(test_action ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_action
#       ${PROJECT_NAME}_types_lib
#       ${PROJECT_NAME}_graph_lib
#       ${catkin_LIBRARIES}
#       )

#add_executable(actionRecogniton src/core/ActionRecognition.cpp)
#add_dependencies(actionRecogniton ${catkin_EXPORTED_TARGETS})
#target_link_libraries(actionRecogniton
#        ${PROJECT_NAME}_types_lib
#        ${PROJECT_NAME}_graph_lib
#        ${catkin_LIBRARIES}
#        )


#add_executable(test_action2 src/tests/test_action2.cpp)
#add_dependencies(test_action2 ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_action2
#        ${PROJECT_NAME}_utils_lib
#        ${catkin_LIBRARIES}
#        )
#
#add_executable(test_feeder src/tests/test_feeder.cpp)
#add_dependencies(test_feeder ${catkin_EXPORTED_TARGETS})
#target_link_libraries(test_feeder
#        ${PROJECT_NAME}_feed_lib
#        ${PROJECT_NAME}_reader_lib
#        ${PROJECT_NAME}_builder_lib
#        ${PROJECT_NAME}_utils_lib
#        )


add_executable(recognition src/nodes/recognition.cpp)
add_dependencies(recognition ${catkin_EXPORTED_TARGETS})
target_link_libraries(recognition
        ${PROJECT_NAME}_interface_lib
        ${catkin_LIBRARIES}
        ${ontologenius_LIBRARIES}
)

add_executable(recognition_multi src/nodes/recognition_multi.cpp)
add_dependencies(recognition_multi ${catkin_EXPORTED_TARGETS})
target_link_libraries(recognition_multi
        ${PROJECT_NAME}_interface_lib
        ${catkin_LIBRARIES}
        ${ontologenius_LIBRARIES}
)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
