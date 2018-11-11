/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    RandomPlanner.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  RandomPlanner class definition of methods.
 */

#include "RandomPlanner.hpp"
#include <vector>
#include <utility>

using std::vector;
using std::pair;
using std::make_pair;

RandomPlanner::RandomPlanner() {
  // Initializing values to the attributes of Map class
  noOfNodes = 0;
  maxStepNumber = 0;
  time = 0;
}

RandomPlanner::~RandomPlanner() {
  // Destructor stub
}

void RandomPlanner::initMap() {

}

vector<pair<int, int> > RandomPlanner::search(vector<vector<int> > worldState,
                               pair<int, int> robotPose,
                                              pair<int, int> goalPose) {
  return path;
}
