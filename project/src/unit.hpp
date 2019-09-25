#ifndef UNIT_HPP
#define UNIT_HPP

#include <iostream>
#include <vector>
#include "point.hpp"

class Unit {
 public:
  Unit() {}
  Unit(const std::vector<Point> &code);

  void outDebug(const std::string &msg);
  void debugCode();
  void updateStatus();
  void calJudgeValue();
  void outToFile(std::string fileName);

 public:
  std::vector<Point> code;
  int codeLen = 0;
  double verticalTolFixNum = 0;
  double levelTolFixNum = 0;
  double tolDistance = 0;
  int fixCount = 0;
  bool isLegal = true;
  double judgeValue = 0;
};

Unit::Unit(const std::vector<Point> &code) {
  this->code = code;
  this->codeLen = code.size();
  this->updateStatus();
}

void Unit::outDebug(const std::string &msg) {
  std::cout << "[" << msg << "]";
  std::cout << " [总距离: " << tolDistance;
  std::cout << ", 校正次数: " << fixCount;

  if (verticalTolFixNum == 0) {
    std::cout << ", 垂直误差: " << verticalTolFixNum + codeLen * theta;
  } else {
    std::cout << ", 垂直误差: " << verticalTolFixNum;
  }

  if (levelTolFixNum == 0) {
    std::cout << ", 水平误差: " << levelTolFixNum + codeLen * theta;
  } else {
    std::cout << ", 水平误差: " << levelTolFixNum;
  }

  // std::cout << ", 水平误差: " << levelTolFixNum + codeLen * theta;
  std::cout << ", 路径长度: " << codeLen;
  std::cout << ", 可行解: " << (isLegal ? 1 : 0);
  std::cout << "]\n";
}

void Unit::debugCode() {
  int index = 0;
  for (auto &v : this->code) {
    v.outDebug(std::to_string(index++));
  }
}

void Unit::outToFile(std::string fileName) {
  static int index = 0;
  std::string file = "../data/" + fileName;
  std::ofstream fout(file);
  fout << std::fixed << std::setprecision(2);

  fout << "[总距离: " << tolDistance;
  fout << ", 校正次数: " << fixCount;
  fout << ", 垂直误差: " << verticalTolFixNum;
  fout << ", 水平误差: " << levelTolFixNum;
  fout << "]\n";

  for (auto &p : code) {
    fout << p.fixIndex << " " << p.x << " " << p.y << " " << p.z << " "
         << p.fixLabel << " " << p.fixError << "\n";
  }

  fout.close();
}
/*
void Unit::updateStatus() {
  this->codeLen = code.size();

  if (code.empty()) {
    return;
  }

  this->verticalTolFixNum = 0;
  this->levelTolFixNum = 0;
  this->code[0].verticalFixNum = 0;
  this->code[0].levelFixNum = 0;
  this->tolDistance = 0;
  this->fixCount = 0;

  for (int i = 1; i < this->codeLen; i++) {
    const Point &pre = code[i - 1];

    Point &now = this->code[i];

    double dis;
    if (i == 1) {
      dis = pre.getDis(pre, now);
    } else {
      const Point &preP1 = code[i - 1];
      dis = pre.getDis(preP1, now);
    }
    double err = dis * delta;

    now.preDis = dis;

    if (now.fixLabel == 1) {
      if (pre.verticalFixNum + err <= a1 && pre.levelFixNum + err <= a2) {
        this->fixCount++;
        now.verticalFixNum = 0;
        now.levelFixNum = pre.levelFixNum + err;
      } else {
        now.verticalFixNum = pre.verticalFixNum + err;
        now.levelFixNum = pre.levelFixNum + err;
        isLegal = false;
      }

    } else if (now.fixLabel == 0) {
      if (pre.verticalFixNum + err <= b1 && pre.levelFixNum + err <= b2) {
        this->fixCount++;
        now.verticalFixNum = pre.verticalFixNum + err;
        now.levelFixNum = 0;
      } else {
        now.verticalFixNum = pre.verticalFixNum + err;
        now.levelFixNum = pre.levelFixNum + err;
        isLegal = false;
      }
    } else {
      now.verticalFixNum = pre.verticalFixNum + err;
      now.levelFixNum = pre.levelFixNum + err;
    }

    this->verticalTolFixNum = now.verticalFixNum;
    this->levelTolFixNum = now.levelFixNum;
    this->tolDistance += dis;
  }

  if (verticalTolFixNum > theta || levelTolFixNum > theta) {
    isLegal = false;
  }
  if (code[code.size() - 1].fixLabel != -66) {
    isLegal = false;
  }
}
*/

void Unit::updateStatus() {
  this->codeLen = code.size();

  if (code.empty()) {
    return;
  }

  this->verticalTolFixNum = 0;
  this->levelTolFixNum = 0;
  this->code[0].verticalFixNum = 0;
  this->code[0].levelFixNum = 0;
  this->tolDistance = 0;
  this->fixCount = 0;

  for (int i = 1; i < this->codeLen; i++) {
    const Point &pre = code[i - 1];

    Point &now = this->code[i];

    double dis;
    if (i == 1) {
      dis = pre.getDis(pre, now);
    } else {
      const Point &preP1 = code[i - 1];
      dis = pre.getDis(preP1, now);
    }
    double err = dis * delta;

    now.preDis = dis;

    if (now.fixLabel == 1) {
      if (pre.verticalFixNum + err <= a1 && pre.levelFixNum + err <= a2) {
        double mrd = 0;
        for (int k = 0; k < 4; k++) {
          mrd += randomDouble(0, 1.0);
        }
        mrd /= 4.0;

        if (problemId == "pro1" || problemId == "pro2" || mrd <= 0.8) {
          this->fixCount++;
          now.verticalFixNum = 0;
          now.levelFixNum = pre.levelFixNum + err;
        } else {
          if (now.fixError == 0) {
            this->fixCount++;
            now.verticalFixNum = 0;
            now.levelFixNum = pre.levelFixNum + err;
          } else {
            if (pre.verticalFixNum + err <= 5) {
              now.ifFix = false;
              now.verticalFixNum = pre.verticalFixNum + err;
              now.levelFixNum = pre.levelFixNum + err;
            } else {
              this->fixCount++;
              now.verticalFixNum = 5;
              now.levelFixNum = pre.levelFixNum + err;
            }
          }
        }
      } else {
        now.verticalFixNum = pre.verticalFixNum + err;
        now.levelFixNum = pre.levelFixNum + err;
        isLegal = false;
      }

    } else if (now.fixLabel == 0) {
      if (pre.verticalFixNum + err <= b1 && pre.levelFixNum + err <= b2) {
        double mrd = 0;
        for (int k = 0; k < 4; k++) {
          mrd += randomDouble(0, 1.0);
        }
        mrd /= 4.0;

        if (problemId == "pro1" || problemId == "pro2" || mrd <= 0.8) {
          this->fixCount++;
          now.verticalFixNum = pre.verticalFixNum + err;
          now.levelFixNum = 0;
        } else {
          if (now.fixError == 0) {
            this->fixCount++;
            now.verticalFixNum = pre.verticalFixNum + err;
            now.levelFixNum = 0;
          } else {
            if (pre.levelFixNum + err <= 5) {
              now.ifFix = false;
              now.verticalFixNum = pre.verticalFixNum + err;
              now.levelFixNum = pre.levelFixNum + err;
            } else {
              this->fixCount++;
              now.verticalFixNum = pre.verticalFixNum + err;
              now.levelFixNum = 5;
            }
          }
        }
      } else {
        now.verticalFixNum = pre.verticalFixNum + err;
        now.levelFixNum = pre.levelFixNum + err;
        isLegal = false;
      }
    } else {
      now.verticalFixNum = pre.verticalFixNum + err;
      now.levelFixNum = pre.levelFixNum + err;
    }

    this->verticalTolFixNum = now.verticalFixNum;
    this->levelTolFixNum = now.levelFixNum;
    this->tolDistance += dis;
  }

  if (verticalTolFixNum > theta || levelTolFixNum > theta) {
    isLegal = false;
  }
  if (code[code.size() - 1].fixLabel != -66) {
    isLegal = false;
  }
}

void Unit::calJudgeValue() {
  judgeValue = tolDistance * 0.001 + (verticalTolFixNum + levelTolFixNum) * 10;
  // this->judgeValue = std::max(verticalTolFixNum, levelTolFixNum);
}

#endif
