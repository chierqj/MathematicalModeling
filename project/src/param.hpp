#ifndef PARAM_HPP
#define PARAM_HPP

// const double a1 = 25;
// const double a2 = 15;
// const double b1 = 20;
// const double b2 = 25;
// const double theta = 30;
// const double delta = 0.001;
// const double a1 = 20;
// const double a2 = 10;
// const double b1 = 15;
// const double b2 = 20;
// const double theta = 20;
// const double delta = 0.001;

double a1, a2, b1, b2, theta, delta;

// 迭代次数
const int TStep = 20000;

// 种群大小
const int PopuSize = 500;

// 交叉概率
const double PC = 0.8;

// 变异概率
const double PM = 0.2;

// 精英保留
const int SaveCount = 2;

// 锦标赛选择
const int RANKN = 3;

// 局部搜索多少个个体
const int PartSearchCount = 30;

std::string problemId;
std::string targedId;

#endif