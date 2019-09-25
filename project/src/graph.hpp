#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <functional>
#include <queue>
#include <set>
#include <vector>
#include "helper.hpp"
#include "param.hpp"
#include "point.hpp"

class Graph {
 public:
  Graph() {}
  void createGraph(const Point &startPoint, const Point &endPoint,
                   const std::vector<Point> &fixSet);

  // 路径最短情况下可行路径(满足约束)
  std::vector<Point> findShortDisPath(const Point &mStart, const Point &mEnd);

  // 校正点最少情况下可行路径(满足约束)
  std::vector<Point> findMinFixCountPath(const Point &mStart,
                                         const Point &mEnd);

  // 初始化种群,随机选取简单路径 (不一定满足约束)
  std::vector<Point> randomSimplePath(const Point &mStart, const Point &mEnd);

  // 简单路径(不一定满足约束)
  std::vector<Point> simplePath(const Point &mStart, const Point &mEnd);

  // void dfs(int u, int fa, int &ok, std::vector<int> &path,std::vector<bool>
  // &vis);

 public:
  std::vector<std::vector<int>> graph;
  int startIndex = 0;
  int endIndex = 0;
  int edgeCount = 0;
  std::set<int> vertix;
  std::vector<Point> indexPoint;

  int feasibleCount = 0;
  int limitFeasibleCount = 0;
  std::vector<std::vector<int>> allPath;

  double minEdge = 1000000000;
  double maxEdge = 0;
};

void Graph::createGraph(const Point &startPoint, const Point &endPoint,
                        const std::vector<Point> &fixSet) {
  std::cout << "> 建图......\n";

  this->graph.resize(endPoint.fixIndex + 7);
  indexPoint.resize(endPoint.fixIndex + 7);

  indexPoint[startPoint.fixIndex] = startPoint;
  indexPoint[endPoint.fixIndex] = endPoint;

  startIndex = startPoint.fixIndex;
  endIndex = endPoint.fixIndex;

  for (auto &p1 : fixSet) {
    for (auto &p2 : fixSet) {
      if (p1.fixIndex == p2.fixIndex) {
        continue;
      }
      double dis = p1.getDis(p1, p2);
      double e = dis * delta;
      if (e <= theta) {
        graph[p1.fixIndex].emplace_back(p2.fixIndex);
        edgeCount++;
        vertix.insert(p1.fixIndex);
        vertix.insert(p2.fixIndex);

        minEdge = std::min(minEdge, dis);
        maxEdge = std::max(maxEdge, dis);
      }
    }
  }

  for (auto p : fixSet) {
    indexPoint[p.fixIndex] = p;
  }

  for (auto &p : fixSet) {
    double dis = startPoint.getDis(startPoint, p);
    double e = dis * delta;
    if (e <= theta) {
      graph[startPoint.fixIndex].emplace_back(p.fixIndex);
      edgeCount++;
      vertix.insert(startPoint.fixIndex);
      vertix.insert(p.fixIndex);

      minEdge = std::min(minEdge, dis);
      maxEdge = std::max(maxEdge, dis);
    }
    dis = p.getDis(p, endPoint);
    e = dis * delta;
    if (e <= theta) {
      graph[p.fixIndex].emplace_back(endPoint.fixIndex);
      edgeCount++;
      vertix.insert(endPoint.fixIndex);
      vertix.insert(p.fixIndex);

      minEdge = std::min(minEdge, dis);
      maxEdge = std::max(maxEdge, dis);
    }
  }

  std::cout << "[地图] [";
  std::cout << "点数: " << vertix.size();
  std::cout << ", 边数: " << edgeCount;
  std::cout << ", 最小边长: " << minEdge;
  std::cout << ", 最大边长: " << maxEdge;
  std::cout << "]\n";

  int maxSz = 0;
  for (auto u : graph) {
    maxSz = std::max(maxSz, (int)u.size());
  }
  std::cout << "maxSz: " << maxSz << "\n";


  // for (auto v : graph[485]) {
  //   std::cout << v << " \n";
  // }
  // std::cout << "\n";
}

std::vector<Point> Graph::findShortDisPath(const Point &mStart,
                                           const Point &mEnd) {
  // std::cout << "> 最短轨迹长度......\n";

  struct Node {
    int u;
    int preP;
    int fixCount;
    double val;
    double verticalError;
    double levelError;
    bool operator<(const Node &r) const {
      if (val == r.val) {
        return fixCount > r.fixCount;
      }
      return val > r.val;
    }
  };

  std::vector<bool> vis(endIndex + 7, false);
  std::vector<int> dis(endIndex + 7, -1);
  std::vector<double> dis1(endIndex + 7, -1);
  std::vector<int> pre(endIndex + 7, -1);

  std::priority_queue<Node> Q;
  Q.push(Node{mStart.fixIndex, mStart.fixIndex, 0, 0, 0, 0});
  dis[mStart.fixIndex] = 0;
  dis1[mStart.fixIndex] = 0;

  while (!Q.empty()) {
    Node h = Q.top();
    Q.pop();

    int u = h.u;

    if (vis[u]) {
      continue;
    }
    vis[u] = true;

    for (auto &v : graph[u]) {
      if (vis[v]) {
        continue;
      }

      const Point &p1 = indexPoint[u];
      const Point &p2 = indexPoint[v];

      int cost = 1;
      double d = p1.getDis(indexPoint[h.preP], p2);
      double e = d * delta;

      double nowVerticalError = h.verticalError + e;
      double nowLevellError = h.levelError + e;

      auto goNext = [&]() {
        if (dis1[v] == -1 || (dis1[u] + e) < dis1[v]) {
          dis1[v] = dis1[u] + e;

          double mrd = 0;
          for (int k = 0; k < 4; k++) {
            mrd += randomDouble(0, 1.0);
          }
          mrd /= 4.0;

          if (problemId == "pro1" || problemId == "pro2" || mrd <= 0.8) {
            if (p2.fixLabel == 1) {
              nowVerticalError = 0;
            }
            if (p2.fixLabel == 0) {
              nowLevellError = 0;
            }
          } else {
            if (p2.fixLabel == 1) {
              if (p2.fixError == 1) {
                nowVerticalError = std::min(nowVerticalError, 5.0);
              } else {
                nowVerticalError = 0;
              }
            }
            if (p2.fixLabel == 0) {
              if (p2.fixError == 1) {
                nowLevellError = std::min(nowLevellError, 5.0);
              } else {
                nowLevellError = 0;
              }
            }
          }

          if (dis[u] + cost < dis[v] || dis[v] == -1) {
            dis[v] = dis[u] + cost;
          }

          Q.push(Node{v, u, dis[v], dis1[v], nowVerticalError, nowLevellError});
          pre[v] = u;
        }
      };

      if (v == mEnd.fixIndex) {
        if (nowVerticalError <= theta && nowLevellError <= theta) {
          goNext();
        }
      } else {
        if (p2.fixLabel == 1 &&
            (nowVerticalError <= a1 && nowLevellError <= a2)) {
          goNext();
        }
        if (p2.fixLabel == 0 &&
            (nowVerticalError <= b1 && nowLevellError <= b2)) {
          goNext();
        }
      }
    }
  }

  int e = mEnd.fixIndex;
  std::vector<Point> shortPath;
  while (e != -1) {
    shortPath.emplace_back(indexPoint[e]);
    e = pre[e];
  }
  reverse(shortPath.begin(), shortPath.end());
  return shortPath;
}

std::vector<Point> Graph::findMinFixCountPath(const Point &mStart,
                                              const Point &mEnd) {
  // std::cout << "> 最少校正点......\n";

  struct Node {
    int u;
    int preP;
    int fixCount;
    double val;
    double verticalError;
    double levelError;
    bool operator<(const Node &r) const {
      if (fixCount == r.fixCount) {
        return val > r.val;
      }
      return fixCount > r.fixCount;
    }
  };

  std::vector<bool> vis(endIndex + 7, false);
  std::vector<int> dis(endIndex + 7, -1);
  std::vector<double> dis1(endIndex + 7, -1);
  std::vector<int> pre(endIndex + 7, -1);

  std::priority_queue<Node> Q;
  Q.push(Node{mStart.fixIndex, mStart.fixIndex, 0, 0, 0, 0});
  dis[mStart.fixIndex] = 0;
  dis1[mStart.fixIndex] = 0;

  while (!Q.empty()) {
    Node h = Q.top();
    Q.pop();

    int u = h.u;

    if (vis[u]) {
      continue;
    }
    vis[u] = true;

    for (auto &v : graph[u]) {
      if (vis[v]) {
        continue;
      }

      const Point &p1 = indexPoint[u];
      const Point &p2 = indexPoint[v];

      int cost = 1;
      double d = p1.getDis(indexPoint[h.preP], p2);
      double e = d * delta;

      double nowVerticalError = h.verticalError + e;
      double nowLevellError = h.levelError + e;

      auto goNext = [&]() {
        if (dis[v] == -1 || (dis[u] + cost) < dis[v]) {
          dis[v] = dis[u] + cost;

          double mrd = 0;
          for (int k = 0; k < 4; k++) {
            mrd += randomDouble(0, 1.0);
          }
          mrd /= 4.0;

          if (problemId == "pro1" || problemId == "pro2" || mrd <= 0.8) {
            if (p2.fixLabel == 1) {
              nowVerticalError = 0;
            }
            if (p2.fixLabel == 0) {
              nowLevellError = 0;
            }
          } else {
            if (p2.fixLabel == 1) {
              if (p2.fixError == 1) {
                nowVerticalError = std::min(nowVerticalError, 5.0);
              } else {
                nowVerticalError = 0;
              }
            }
            if (p2.fixLabel == 0) {
              if (p2.fixError == 1) {
                nowLevellError = std::min(nowLevellError, 5.0);
              } else {
                nowLevellError = 0;
              }
            }
          }

          if (dis1[u] + d < dis1[v] || dis1[v] == -1) {
            dis1[v] = dis1[u] + d;
          }

          Q.push(Node{v, u, dis[v], dis1[v], nowVerticalError, nowLevellError});
          pre[v] = u;
        }
      };

      if (v == mEnd.fixIndex) {
        if (nowVerticalError <= theta && nowLevellError <= theta) {
          goNext();
        }
      } else {
        if (p2.fixLabel == 1 &&
            (nowVerticalError <= a1 && nowLevellError <= a2)) {
          goNext();
        }
        if (p2.fixLabel == 0 &&
            (nowVerticalError <= b1 && nowLevellError <= b2)) {
          goNext();
        }
      }
    }
  }

  int e = mEnd.fixIndex;
  std::vector<Point> shortPath;
  while (e != -1) {
    shortPath.emplace_back(indexPoint[e]);
    e = pre[e];
  }
  reverse(shortPath.begin(), shortPath.end());
  return shortPath;
}

std::vector<Point> Graph::randomSimplePath(const Point &mStart,
                                           const Point &mEnd) {
  int ok = 0;
  std::vector<bool> vis(endIndex + 7, false);
  std::vector<int> pathIndex;
  std::vector<int> tmpPath;

  std::function<void(int u, int fa)> dfs;
  dfs = [&](int u, int fa) {
    if (ok == 1 || u == mEnd.fixIndex) {
      ok = 1;
      pathIndex = tmpPath;
      return;
    }

    int sz = graph[u].size();
    int rd = randomInt(0, sz - 1);

    int count = 0;
    while (rd == fa || vis[graph[u][rd]]) {
      rd = randomInt(0, sz - 1);
      if (count++ >= 20) {
        return;
      }
    }

    int v = graph[u][rd];
    vis[v] = true;
    tmpPath.emplace_back(v);
    dfs(v, u);
    vis[v] = false;
    tmpPath.pop_back();
  };

  vis[mStart.fixIndex] = true;
  tmpPath.emplace_back(mStart.fixIndex);
  dfs(mStart.fixIndex, -1);

  while (!ok || (int)pathIndex.size() < 2) {
    // std::cout << "随机找不到路: " << pathIndex.size() << "\n";
    dfs(mStart.fixIndex, -1);
  }

  if (!ok || (int)pathIndex.size() < 2) {
    std::cout << "随机找不到路: " << pathIndex.size() << "\n";
    dfs(mStart.fixIndex, -1);
  }

  std::vector<Point> ansPath;
  for (auto &v : pathIndex) {
    ansPath.emplace_back(indexPoint[v]);
  }
  return ansPath;
}

std::vector<Point> Graph::simplePath(const Point &mStart, const Point &mEnd) {
  int ok = 0;
  std::vector<bool> vis(endIndex + 7, false);
  std::vector<int> pathIndex;

  std::function<void(int u, int fa)> dfs;
  dfs = [&](int u, int fa) {
    if (ok == 1 || u == mEnd.fixIndex) {
      ok = 1;
      return;
    }

    int sz = graph[u].size();
    int rd = randomInt(0, sz - 1);

    int count = 0;
    while (rd == fa || vis[graph[u][rd]]) {
      rd = randomInt(0, sz - 1);
      if (count++ >= 20) {
        return;
      }
    }

    int v = graph[u][rd];
    vis[v] = true;
    pathIndex.emplace_back(v);
    dfs(v, u);
    vis[v] = false;
    pathIndex.pop_back();
  };
  std::vector<Point> ansPath;
  for (auto &v : pathIndex) {
    ansPath.emplace_back(indexPoint[v]);
  }
  return ansPath;
}

#endif