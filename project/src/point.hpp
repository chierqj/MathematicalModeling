#ifndef POINT_HPP
#define POINT_HPP

#include <cmath>
#include <iostream>
#include "helper.hpp"
#include "param.hpp"

class Point {
 public:
  Point() {}
  Point(const double x, const double y, const double z) : x(x), y(y), z(z) {}
  Point(const int fixIndex, const double x, const double y, const double z,
        const int fixLabel, const int fixError)
      : fixIndex(fixIndex),
        x(x),
        y(y),
        z(z),
        fixLabel(fixLabel),
        fixError(fixError) {}

  // 测试结果输出
  void outDebug(std::string msg) const;
  // 向着目标点nxtP移动len个距离后的点
  Point moveToPoint(const Point &nxtP, const double &len);
  // 获取两点距离
  double getDis(const Point &preP, const Point &nxtP) const;
  // 求解平面方程
  // void calFlatEquation(const Point &p1, const Point &p2, const Point P3,
  //                      double &A, double &B, double &C, double &D);
  // // 根据两点求直线方程
  // Line calLine(const Point &p1, const Point &p2);

  // // 过该点，垂直与直线L的垂线方程
  // Line calVerLine(const Line &L);

  // // 两条直线求交点
  // Point calComPoint(const Line &L1, const Line &L2);


  double getDown();
  void shuffle();

 public:
  int fixIndex = -1;
  double x;
  double y;
  double z;
  int fixLabel = -1;
  int fixError = -1;

  double verticalFixNum = 0;
  double levelFixNum = 0;
  double preDis = 0;

  bool ifFix = true;
};

void Point::outDebug(std::string msg) const {
  std::cout << "[" << msg << "] ";
  std::cout << " [校正点: " << fixIndex;
  std::cout << "; 坐标: (" << x << ", " << y << ", " << z;
  std::cout << "); 距离: " << preDis;
  std::cout << "; 误差: (" << verticalFixNum << ", " << levelFixNum;
  std::cout << ") 校正: (" << fixLabel << ", " << fixError << ")]\n";
}

double Point::getDown() { return sqrt(x * x + y * y + z * z); }

Point Point::moveToPoint(const Point &nxtP, const double &len) {
  Point dir(nxtP.x - x, nxtP.y - y, nxtP.z - z);
  double down = dir.getDown();
  Point ret;
  ret.x = x + len * (dir.x / down);
  ret.y = y + len * (dir.y / down);
  ret.z = z + len * (dir.z / down);
  ret.verticalFixNum = verticalFixNum + len * delta;
  ret.levelFixNum = levelFixNum + len * delta;
  return ret;
}

double Point::getDis(const Point &preP, const Point &nxtP) const {
  double sum = std::pow((nxtP.x - x), 2) + std::pow((nxtP.y - y), 2) +
               std::pow((nxtP.z - z), 2);

  // 问题一
  double result = std::sqrt(sum);
  if (problemId == "pro2") {
    result *= 1.04719753;
  }
  // 问题二
  // a = ((p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y));

  // b = ((p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z));

  // c = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));

  // d = (0 - (a * p1.x + b * p1.y + c * p1.z));

  return result;
}

// double Point::getDis(const Point &nxtP) const {
//   return sqrt(std::pow((nxtP.x - x), 2) + std::pow((nxtP.y - y), 2) +
//               std::pow((nxtP.z - z), 2)) *
//          (3.1415926 / 3.0);
// }

void Point::shuffle() {
  double l = 200, r = 1000;
  this->x += randomDouble(l, r);
  this->y += randomDouble(l, r);
  this->z += randomDouble(l, r);
  this->fixLabel = -1;
  this->fixError = -1;
}
/*
  double x;
  double y;
  double z;
  double verticalFixNum = 0;
  double levelFixNum = 0;
  int fixLabel = -1;
  int fixError = -1;
*/

#endif