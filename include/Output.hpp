/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    Output.hpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  Output class declaration.
 */

#ifndef INCLUDE_OUTPUT_HPP_
#define INCLUDE_OUTPUT_HPP_

#include <vector>
#include <utility>
#include <string>

/**
 * @brief Declaring class attributes and methods
 */
class Output {
 public:
  /**
   * @brief Constructor
   */
  Output();

  /**
   * @brief Destructor
   */
  virtual ~Output();

  /**
   *   @brief Write the trajectory/path found by the planner in a text file
   *   @param vector of pair of int value of the path given by the planner
   *   @return none
   */
  void writeTextFile(std::vector<std::pair<int, int> > path);

  /**
   *   @brief Output the trajectory
   *   @param none
   *   @return none
   */
  void showOutput(std::vector<std::pair<int, int> > path);

  std::string location;
};

#endif  // INCLUDE_OUTPUT_HPP_
