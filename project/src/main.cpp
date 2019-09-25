#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include "param.hpp"
#include "point.hpp"
#include "simulation.hpp"
#include "unit.hpp"

void readFile(std::string fileName, Point &startPoint, Point &endPoint,
              std::vector<Point> &fixSet, std::vector<Point> &verticalFixSet,
              std::vector<Point> &levelFixSet) {
  std::cout << "> 读文件......\n";
  std::string path = "../data/";
  path += fileName;
  std::cout << path << "\n";
  std::ifstream fin(path, std::ios::in);
  std::string line;
  while (std::getline(fin, line)) {
    int rfi;
    double rx, ry, rz;
    int rfl, rfe;
    std::stringstream ss(line);
    ss >> rfi >> rx >> ry >> rz >> rfl >> rfe;
    Point p(rfi, rx, ry, rz, rfl, rfe);
    if (rfl == -6) {
      startPoint = p;
    } else if (rfl == -66) {
      endPoint = p;
    } else {
      if (rfl == 1) {
        verticalFixSet.emplace_back(p);
      } else {
        levelFixSet.emplace_back(p);
      }
      fixSet.emplace_back(p);
    }
  }
  fin.close();

  int sz = fixSet.size();
  startPoint.fixIndex = sz + 6;
  endPoint.fixIndex = sz + 66;

  std::cout << "垂直: " << verticalFixSet.size() << "\n";
  std::cout << "水平: " << levelFixSet.size() << "\n";

  startPoint.outDebug("起点");
  endPoint.outDebug("终点");
}

void execute(std::string fileName) {
  Point startPoint, endPoint;
  std::vector<Point> fixSet;
  std::vector<Point> verticalFixSet;
  std::vector<Point> levelFixSet;
  readFile(fileName, startPoint, endPoint, fixSet, verticalFixSet, levelFixSet);

  int t = 0;
  Simulation simu(startPoint, endPoint, fixSet, verticalFixSet, levelFixSet);
  simu.execute();
}

int main(int argc, char *argv[]) {
  auto _start = std::chrono::high_resolution_clock::now();
  std::cout << std::fixed << std::setprecision(2);

  std::string fileName = argv[1];
  std::string pId = argv[2];
  problemId = pId;
  targedId = argv[3];
  std::cout << fileName << "\n";
  std::cout << problemId << "\n";
  std::cout << targedId << "\n";

  if (fileName == "file1.txt") {
    a1 = 25;
    a2 = 15;
    b1 = 20;
    b2 = 25;
    theta = 30;
    delta = 0.001;
  } else {
    a1 = 20;
    a2 = 10;
    b1 = 15;
    b2 = 20;
    theta = 20;
    delta = 0.001;
  }

  execute(fileName);

  auto _end = std::chrono::high_resolution_clock::now();
  std::cout << "[运行时间] ["
            << std::chrono::duration<double, std::milli>(_end - _start).count()
            << " ms]\n";
  return 0;
}