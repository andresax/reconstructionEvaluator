/*
 * Main.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: andrea
 */

#include <Configuration.h>

int main(int argc, char **argv) {


  reconstructorEvaluator::Configuration conf("/home/andrea/workspaceC/reconstructionEvaluator/config/fountain.config");
  conf.parse();

}

