/*
 * Main.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: andrea
 */

#include <GtComparator.h>


int main(int argc, char **argv) {


  reconstructorEvaluator::GtComparator comparator("/home/andrea/workspaceC/reconstructionEvaluator/config/fountain.config");
  comparator.run();

}

