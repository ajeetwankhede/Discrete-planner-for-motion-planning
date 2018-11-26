/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    OutputTest.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  Test stubs for class Output.
 */

#include <iostream>
#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <utility>
#include <memory>
#include <fstream>
#include "Output.hpp"

std::shared_ptr<Output> outputTestObject;

/**
 * @brief Check if writeTextFile method works
 */
TEST(Output, writeTextFileTest) {
  outputTestObject = std::make_shared<Output>();
  std::vector<std::pair<int, int> > path;
  path.push_back(std::make_pair(1, 1));
  // Call the method to write a text file
  outputTestObject->writeTextFile(path);
  //outputTestObject->location =
  //      "../output/path.txt";
  // Create a isftream object to handle text file
  std::ifstream myFile(outputTestObject->location);
  std::string line;
  // Check if the file is opened successfully
  if (myFile.is_open()) {
    // Get the first line from the text file
    getline(myFile, line);
    // Get the second line from the text file
    getline(myFile, line);
    // Get the third line from the text file
    getline(myFile, line);
    //std::cout << line << std::endl;
    myFile.close();
  } else {
    line = "Unable to open file";
  }
  EXPECT_EQ("1\t1", line);
}
