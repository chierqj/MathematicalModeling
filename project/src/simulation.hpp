#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <cassert>
#include <fstream>
#include <set>
#include <vector>
#include "graph.hpp"
#include "helper.hpp"
#include "param.hpp"
#include "point.hpp"
#include "unit.hpp"

class Simulation {
 public:
  Simulation() {}
  Simulation(const Point &startPoint, const Point &endPoint,
             const std::vector<Point> &fixSet,
             const std::vector<Point> &verticalFixSet,
             const std::vector<Point> &levelFixSet)
      : startPoint(startPoint),
        endPoint(endPoint),
        fixSet(fixSet),
        verticalFixSet(verticalFixSet),
        levelFixSet(levelFixSet) {}

  // 单元测试
  bool test();
  // 入口函数
  void execute();
  // 种群初始化
  void popuInit();
  // 交叉
  void cross();
  // 变异
  void variation();
  // 校正
  void fix();
  // 局部搜索
  void partSearch();
  // 选择
  void select();
  // 迭代到下一代的处理
  void initialize();


 public:
  Point startPoint;
  Point endPoint;
  std::vector<Point> fixSet;
  std::vector<Point> verticalFixSet;
  std::vector<Point> levelFixSet;
  std::vector<Unit> population;
  std::vector<Unit> sons;
  std::vector<Unit> best;
  Graph graphObj;
};

void Simulation::popuInit() {
  std::cout << "> 初始化种群......\n";

  std::vector<Point> path;

  // 最短轨迹长度一个
  path = graphObj.findShortDisPath(startPoint, endPoint);
  Unit u1(path);
  population.emplace_back(u1);

  // 最少校正点一个
  path = graphObj.findMinFixCountPath(startPoint, endPoint);
  Unit u2(path);
  population.emplace_back(u2);

  // 随机其他的
  for (int i = 2; i < PopuSize; i++) {
    path = graphObj.randomSimplePath(startPoint, endPoint);
    Unit u(path);
    population.emplace_back(u);
  }

  // int index = 0;
  // for (auto &unit : population) {
  //   unit.outDebug("个体 " + std::to_string(index++));
  // }
}

void Simulation::cross() {
  std::cout << "> 交叉......\n";
  this->sons.clear();
  for (int i = 1; i < PopuSize; i += 2) {
    Unit &u1 = population[i - 1];
    Unit &u2 = population[i];

    double rd = randomDouble(0, 1.0);
    if (rd > PC) {
      this->sons.emplace_back(u1);
      this->sons.emplace_back(u2);
      continue;
    }

    int len = std::min(u1.codeLen, u2.codeLen);
    int l = randomInt(0, len - 1), r = randomInt(0, len - 1);
    if (l > r) {
      std::swap(l, r);
    }

    std::vector<Point> sc1, sc2;
    for (int j = 0; j < l; j++) {
      sc1.emplace_back(u1.code[j]);
      sc2.emplace_back(u2.code[j]);
    }
    for (int j = l; j <= r; j++) {
      sc1.emplace_back(u2.code[j]);
      sc2.emplace_back(u1.code[j]);
    }
    for (int j = r + 1; j < u1.codeLen; j++) {
      sc1.emplace_back(u1.code[j]);
    }
    for (int j = r + 1; j < u2.codeLen; j++) {
      sc2.emplace_back(u2.code[j]);
    }
    Unit son1(sc1), son2(sc2);
    this->sons.emplace_back(son1);
    this->sons.emplace_back(son2);
  }
}

void Simulation::variation() {
  std::cout << "> 变异......\n";

  for (auto &unit : sons) {
    int index = -1;
    for (auto &p : unit.code) {
      ++index;
      if (index == 0 || index == unit.codeLen - 1) {
        continue;
      }

      double rd = randomDouble(0, 1.0);
      if (rd > PM) {
        continue;
      }
      int rdIndex;
      if (graphObj.graph[p.fixIndex].size() >= 1) {
        rdIndex = randomInt(0, graphObj.graph[p.fixIndex].size() - 1);
        p = graphObj.indexPoint[graphObj.graph[p.fixIndex][rdIndex]];
      } else {
        rdIndex = randomInt(0, (int)fixSet.size() - 1);
        p = fixSet[rdIndex];
      }
    }
    unit.updateStatus();
  }
}

void Simulation::fix() { std::cout << "> 校正......\n"; }

void Simulation::partSearch() {
  std::cout << "> 局部搜索......\n";

  std::set<int> st;
  while (true) {
    if ((int)st.size() > PartSearchCount) {
      break;
    }
    int rd = randomInt(0, PopuSize - 1);
    st.insert(rd);
  }

  for (auto &index : st) {
    Unit &unit = sons[index];
    if (unit.isLegal) {
      continue;
    }
    // 从useP开始，后面就不满足约束了
    int useP = 0;
    while (useP < unit.codeLen) {
      const Point &p = unit.code[useP];
      if (p.verticalFixNum > theta || p.levelFixNum > theta) {
        break;
      }
      useP++;
    }
    if (useP >= unit.codeLen) {
      continue;
    }
    assert(useP > 0);

    // 不满足约束的折半处理m是中点
    int m = randomInt(useP, unit.codeLen - 1);

    const Point &p1 = unit.code[useP - 1];
    const Point &pm = unit.code[m];
    const Point &p2 = endPoint;

    std::vector<Point> path1;
    if (targedId == "tar1") {
      path1 = graphObj.findShortDisPath(p1, p2);
      
    } else {
      path1 = graphObj.findMinFixCountPath(p1, p2);
    }

    if (m < unit.codeLen - 1) {
      std::vector<Point> path2;

      if (targedId == "tar1") {
        path2 = graphObj.findShortDisPath(pm, p2);
      } else {
        path2 = graphObj.findMinFixCountPath(pm, p2);
      }
      path1.pop_back();
      path1.insert(path1.end(), path2.begin(), path2.end());
    }

    std::vector<Point> newPath;
    for (int i = 0; i < useP - 1; i++) {
      newPath.emplace_back(unit.code[i]);
    }
    newPath.insert(newPath.end(), path1.begin(), path1.end());

    // std::cout <<
    // "--------------------------------------------------------\n";

    // unit.outDebug("个体 A");
    // unit.debugCode();

    // std::cout << "\n(" << p1.fixIndex << ", " << pm.fixIndex << ", "
    //           << p2.fixIndex << ")\n";

    // if (newPath[newPath.size() - 1].fixLabel != -66) {
    //   std::cout << "path error\n";
    //   unit.debugCode();
    //   exit(0);
    // }

    unit.code = newPath;
    unit.updateStatus();

    // unit.outDebug("个体 B");
    // unit.debugCode();
    // std::cout << "\n";
  }
}

void Simulation::select() {
  std::cout << "> 选择......\n";

  std::vector<Unit> allUnits = population;
  allUnits.insert(allUnits.end(), sons.begin(), sons.end());
  for (auto &unit : allUnits) {
    unit.calJudgeValue();
  }

  // 校验有问题，没判断是否可行解;要看能不能经过校正点
  sort(allUnits.begin(), allUnits.end(), [&](const Unit &u1, const Unit &u2) {
    if (u1.isLegal && u2.isLegal) {
      if (u1.tolDistance == u2.tolDistance) {
        return u1.codeLen < u2.codeLen;
      }
      if (u1.codeLen == u2.codeLen) {
        return u1.tolDistance < u2.tolDistance;
      }
      if (targedId == "tar1") {
        return u1.tolDistance < u2.tolDistance;
      } else {
        return u1.codeLen < u2.codeLen;
      }
    }
    if (u1.isLegal) {
      return true;
    }
    if (u2.isLegal) {
      return false;
    }
    return u1.judgeValue < u2.judgeValue;
  });

  int tolSz = allUnits.size();
  assert(SaveCount <= tolSz);

  best.clear();
  for (int i = 0; i < SaveCount; i++) {
    best.emplace_back(allUnits[i]);
  }

  for (int i = 0; i < PopuSize - SaveCount; i++) {
    Unit u;
    for (int j = 0; j < RANKN; j++) {
      int rd = randomInt(0, tolSz - 1);
      if (j == 0) {
        u = allUnits[rd];
        continue;
      }

      if (u.isLegal && allUnits[j].isLegal) {
        if (allUnits[j].tolDistance < u.tolDistance) {
          u = allUnits[j];
        }
      } else if (u.isLegal) {
        continue;
      } else if (allUnits[j].isLegal) {
        u = allUnits[j];
      } else {
        if (allUnits[j].judgeValue < u.judgeValue) {
          u = allUnits[j];
        }
      }
    }
    best.emplace_back(u);
  }
  assert((int)best.size() == PopuSize);
}

void Simulation::initialize() {
  population = best;
  population[0].outDebug("最优");
  population[1].outDebug("最优");
  population[2].outDebug("最优");
}

bool Simulation::test() {
  return false;
  std::cout << "> 测试......\n";

  std::vector<Point> testPath;
  for (int i = 0; i < 100; i++) {
    testPath = graphObj.randomSimplePath(startPoint, endPoint);
    // std::cout << testPath.size() << "\n";
  }
  Unit test(testPath);
  // test.outDebug("测试");
  // test.debugCode();
  // graphObj.findFeasiblePath(startPoint, endPoint);
  return true;
}

void Simulation::execute() {
  this->graphObj.createGraph(startPoint, endPoint, fixSet);

  if (Simulation::test()) {
    return;
  }

  // 种群初始化
  popuInit();

  int t = 0;
  while (t < TStep) {
    std::cout << "------------------------Step: " << t
              << "-----------------------------\n";
    t++;
    this->cross();
    this->variation();
    this->fix();
    this->partSearch();
    this->select();
    this->initialize();
  }

  std::vector<Point> path;
  int index = 0;
  Unit u;

  std::cout << "-----------------------最短路径-----------------------------\n";
  u.code = graphObj.findShortDisPath(startPoint, endPoint);
  u.updateStatus();
  u.outDebug("最短路径");
  index = 0;
  for (auto &p : u.code) {
    p.outDebug("路径 " + std::to_string(index++));
  }
  u.outToFile("short_path.txt");

  std::cout << "-----------------------最少校正-----------------------------\n";
  u.code = graphObj.findMinFixCountPath(startPoint, endPoint);
  u.updateStatus();
  u.outDebug("最少校正");
  index = 0;
  for (auto &p : u.code) {
    p.outDebug("路径 " + std::to_string(index++));
  }
  u.outToFile("min_fix_count_path.txt");

  std::cout << "-----------------------遗传算法-----------------------------\n";
  u = best[0];
  for (auto &unit : best) {
    if (unit.tolDistance < u.tolDistance) {
      u = unit;
    }
  }
  u.outDebug("遗传算法");
  index = 0;
  for (auto &p : u.code) {
    p.outDebug("路径 " + std::to_string(index++));
  }
  u.outToFile("ga.txt");
}

#endif