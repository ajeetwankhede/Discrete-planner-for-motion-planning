/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    Output.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/10/2018
 *  @version 1.0
 *  @brief   Discrete planner for motion planning
 *  @section DESCRIPTION
 *  Output class definition of methods.
 */

#include <iostream>
#include <vector>
#include <utility>
#include <fstream>
#include "Output.hpp"

using std::vector;
using std::pair;
using std::cout;
using std::endl;

Output::~Output() {
  // Destructor stub
}

void Output::writeTextFile(vector<pair<int, int> > path) {
  // Save the trajectory coordinates in a text file
  std::ofstream myFile;
  myFile.open(location);
  myFile << "The x and y coordinates of the optimal trajectory are:\n";
  myFile << "X\tY\n";
  for (auto i : path) {
    myFile << i.first << "\t" << i.second << "\n";
  }
  myFile.close();
  cout << "A text file with trajectory coordinates has been saved at: "
       << location << endl;
}

void Output::showOutput(vector<pair<int, int> > path) {
  // Output the trajectory coordinates
  cout << "Number of elements in path are: " << path.size() << endl;
  for (auto i : path) {
    cout << i.first << "\t" << i.second << endl;
  }
}
