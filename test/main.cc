
#include "dp_quadruped_walker/dp_quadruped_walker.h"
#include "dproki/ObjFileReader.h"

using namespace Dp;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    fprintf(stderr, "%s <dir path> <obj path>\n", argv[0]);
    return 1;
  }

  std::string dirpath(argv[1]);
  std::string objpath(argv[2]);

  auto robot = Dp::ObjFileReader::ImportObjFile(dirpath, objpath);
  if (robot == NULL) {
    fprintf(stderr, "fail to load %s %s.\n", dirpath.c_str(), objpath.c_str());
    return 2;
  }

  robot->Setup();
  robot->UpdateCasCoords();

  QuadrupedWalker qwalker(robot);

  QuadrupedAngles& angs = qwalker.Walk({0.05, 0.00, 0.00});

  for (auto ang : angs) {
    fprintf(stdout, " -- %lf, %lf, %lf | %lf, %lf, %lf | %lf, %lf, %lf | %lf, %lf, %lf\n",
        ang.angle[0][0], ang.angle[0][1], ang.angle[0][2],
        ang.angle[1][0], ang.angle[1][1], ang.angle[1][2],
        ang.angle[2][0], ang.angle[2][1], ang.angle[2][2],
        ang.angle[3][0], ang.angle[3][1], ang.angle[3][2]
        );
  }

  return 0;
}

