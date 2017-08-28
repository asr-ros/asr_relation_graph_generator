cmake_minimum_required(VERSION 2.4.6)
project(asr_relation_graph_generator)
add_definitions(-Wall -std=gnu++0x -ggdb)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(cmake_modules)

# include Eigen library
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  asr_msgs
  asr_lib_ism
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES asr_relation_graph_generator
  CATKIN_DEPENDS roscpp asr_msgs
  DEPENDS EIGEN3
)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Build a library containing the relation graph generator.
add_library(asr_relation_graph_generator
		      src/trainer/PSMTrainer.cpp src/trainer/AbstractTrainer.cpp
		      src/trainer/TreeNode.cpp src/trainer/ObjectInformation.cpp
		      src/trainer/source/PbdSceneGraphSource.cpp src/trainer/source/AbstractSource.cpp src/trainer/source/ObjectSetList.cpp src/trainer/source/ObjectSet.cpp src/trainer/source/Object.cpp
		      src/trainer/generator/heuristic/HeuristicalTreeGenerator.cpp src/trainer/generator/heuristic/DirectionRelationHeuristic.cpp src/trainer/generator/heuristic/AbstractHeuristic.cpp
		      src/trainer/generator/AbstractGraphGenerator.cpp
		      src/helper/MathHelper.cpp
		      src/trainer/FullyMeshedTrainer.cpp src/trainer/generator/fully_meshed/FullyMeshedGenerator.cpp
		      src/trainer/TopologyTreeTrainer.cpp src/trainer/generator/topology_tree/TopologyTreeGenerator.cpp
		      src/topology_creator/Relation.cpp
		      
		      src/topology_creator/ConnectivityChecker.cpp 
		      src/topology_creator/TopologyCreator.cpp
		      src/topology_creator/Topology.cpp
)

# dependcy to the message generation of asr_msgs to ensure 
# the message header files exist before trying to compile this package
add_dependencies(${PROJECT_NAME} asr_msgs_generate_messages_cpp)

target_link_libraries(asr_relation_graph_generator
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Build the test program.
add_executable(Tester src/main.cpp)

# dependcy to the message generation of asr_msgs to ensure 
# the message header files exist before trying to compile this package
add_dependencies(Tester asr_msgs_generate_messages_cpp)

target_link_libraries(Tester 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  asr_relation_graph_generator
)
