/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    OptimalPlanner.hpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  OptimalPlanner class declaration.
 */

#ifndef INCLUDE_OPTIMALPLANNER_HPP_
#define INCLUDE_OPTIMALPLANNER_HPP_

#include <map>
#include <vector>
#include <utility>

/**
 * @brief Declaring class attributes and methods
 */
class OptimalPlanner {
 public:
  /**
   * @brief Constructor
   */
  OptimalPlanner();

  /**
   * @brief Destructor
   */
  virtual ~OptimalPlanner();

  /**
   *   @brief Call this method to initialize different variables regarding to the world
   *   @param none
   *   @return none
   */
  void initMap();

  /**
   *   @brief Run the search method to find a path from robot pose to goal pose
   *   @param vector of vector of int containin the world state
   *   @param pair of int of the robot pose or start node
   *   @param pair of int of the goal pose or end node
   *   @return vector of pair of int containing the path found by the search algorithm.
   */
  std::vector<std::pair<int, int> > search(
      std::vector<std::vector<int> > worldState, std::pair<int, int> robotPose,
      std::pair<int, int> goalPose);

  std::vector<std::pair<int, int> > path;

 private:
  int noOfNodes;
  double time;
  std::map<std::pair<int, int>, int> visitedNodes;
  std::map<std::pair<int, int>, double> parentCost;
  std::map<std::pair<int, int>, double> currentCost;
  std::map<std::pair<int, int>, std::pair<int, int> > parentNode;
};

#endif  // INCLUDE_OPTIMALPLANNER_HPP_
