/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    RandomPlanner.hpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  RandomPlanner class declaration.
 */

#ifndef INCLUDE_RANDOMPLANNER_HPP_
#define INCLUDE_RANDOMPLANNER_HPP_

#include <map>
#include <vector>
#include <utility>
#include "Map.hpp"

/**
 * @brief Declaring class attributes and methods
 */
class RandomPlanner {
 public:
  /**
   * @brief Constructor
   */
  RandomPlanner();

  /**
   * @brief Destructor
   */
  virtual ~RandomPlanner();

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
  std::map<std::pair<int, int>, int> visitedNodes;
  std::map<std::pair<int, int>, int> obstacleSpace;
  int maxStepNumber;
  double time;
  // Create an object of class Map
  Map M;

 private:
  int noOfNodes;
};

#endif  // INCLUDE_RANDOMPLANNER_HPP_
