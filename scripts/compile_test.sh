#!/bin/bash

cd /home/avigne/Projets/ArchiThese/catkin_ws
source devel/setup.bash

g++ -std=c++17 -I/home/avigne/Projets/ArchiThese/catkin_ws/devel/include \
    -I/home/avigne/Projets/ArchiThese/catkin_ws/src/Procedural/include \
    -I/home/avigne/Projets/ArchiThese/catkin_ws/src/procedural_interfaces/include \
    -L/home/avigne/Projets/ArchiThese/catkin_ws/devel/lib \
    -lprocedural_memory_lib \
    src/Procedural/scripts/test_priority_parsing.cpp \
    -o test_priority_parsing

echo "Compilation completed."
echo "Run: ./test_priority_parsing"