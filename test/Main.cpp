#include <GtComparator.h>

int main(int argc, char **argv) {

  // reconstructorEvaluator::GtComparator comparator("/home/andrea/workspaceC/reconstructionEvaluator/config/kitti3.config");
  // comparator.run3();
  reconstructorEvaluator::GtComparator comparator("/home/andrea/workspaceC/reconstructionEvaluator/config/fountain.config");
  comparator.run();

}

