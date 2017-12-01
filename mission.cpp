#include "mission.h"
#include "astar.h"
#include "dijkstra.h"
#include "theta.h"
#include "xmllogger.h"
#include "gl_const.h"
#include <iostream>

Mission::Mission()
{
    logger = nullptr;
    search = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    logger = nullptr;
    search = nullptr;
}

Mission::~Mission()
{
    if (logger)
        delete logger;
    if (search)
        delete search;
}

bool Mission::getMap()
{
    return map.getMap(fileName);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::createLog()
{
    if (logger != NULL) delete logger;
    logger = new XmlLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createEnvironmentOptions()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS || config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC]);
    else
        options = EnvironmentOptions(config.SearchParams[CN_SP_AS], config.SearchParams[CN_SP_AD],
                                     config.SearchParams[CN_SP_CC], config.SearchParams[CN_SP_MT]);
}

void Mission::createSearch()
{
    if (search)
        delete search;
//    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS)
//        search = new BFS();
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        search = new Dijkstra();
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        search = new Astar(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_JP_SEARCH)
        search = new JP_Search(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        search = new Theta(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
}

#include <queue>
#include "node.h"
#include <math.h>
#include <vector>
#include <time.h>

double rho(Node v, Node u) {
    double di = v.i - u.i;
    double dj = v.j - u.j;
    return sqrt(di * di + dj * dj);
}

bool operator<(Node v, Node u) {
    return v.F < u.F;
}

void Mission::startSearch()
{
    double t_begin = clock();
    const double INF = 1e9;
    std::vector<std::vector<Node>> *grid0 = new std::vector<std::vector<Node>> {map.getMapWidth(), std::vector<Node>(map.getMapHeight())};
    std::vector<std::vector<Node>>& grid = *grid0;

    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            grid[i][j].i = i;
            grid[i][j].j = j;
            grid[i][j].F = INF;
            grid[i][j].used = false;
        }
    }
    int step_num = 0;
    std::priority_queue<Node*> qu;
    grid[map.start_i][map.start_j].F = 0.;
    qu.push(&grid[map.start_i][map.start_j]);
    while (!qu.empty()) {
        while (!qu.empty() && qu.top()->used) {
            qu.pop();
        }
        if (qu.empty()) {
            break;
        }

        ++step_num;
        Node& v = *qu.top();
        // std::cout << "NODE: " <<  v.i << ' ' << v.j << std::endl;
        v.used = true;
        qu.pop();
        int di[] = {0, 1, 0, -1, 1, 1, -1, -1};
        int dj[] = {-1, 0, 1, 0, 1, -1, 1, -1};
        int dir_num;
        if (options.allowdiagonal) {
            dir_num = 8;
        } else {
            dir_num = 4;
        }
        for (int dir = 0; dir < dir_num; ++dir) {
            int new_i = v.i + di[dir], new_j = v.j + dj[dir];
            if (new_i < 0 || new_j < 0 || new_i >= map.getMapHeight() || new_j > map.getMapWidth()) {
                continue;
            } else if (map.CellIsObstacle(new_i, new_j)) {
                continue;
            }
            Node& u = grid[new_i][new_j];
            int obst_num = map.CellIsObstacle(v.i, u.j) + map.CellIsObstacle(u.i, v.j);
            if ((!options.cutcorners && obst_num > 0) || (!options.allowsqueeze && obst_num > 1)) {
                continue;
            }
            if (u.F > v.F + rho(v, u)) {
                u.F = v.F + rho(v, u);
                u.parent = &v;
                qu.push(&u);
            }
        }
    }

    if (grid[map.goal_i][map.goal_j].F == INF) {
        return;
    }
    sr.pathfound = true;
    sr.pathlength = grid[map.goal_i][map.goal_j].F;

    std::list<Node> *ans = new std::list<Node>();
    Node cur_n = grid[map.goal_i][map.goal_j];
    while (cur_n.i != map.start_i || cur_n.j != map.start_j) {
        std::cout << "NODE: " << cur_n.i << ' ' << cur_n.j << std::endl;
        ans->emplace_front(cur_n);
        cur_n = *(cur_n.parent);
    }
    sr.hppath = ans;
    sr.lppath = ans;
    sr.nodescreated = map.getMapHeight() * map.getMapWidth();
    sr.numberofsteps = step_num;
    sr.time = (clock() - t_begin) / CLOCKS_PER_SEC;
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.getCellSize() << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog()
{
    logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, map.getCellSize());
    if (sr.pathfound) {
        logger->writeToLogPath(*sr.lppath);
        logger->writeToLogHPpath(*sr.hppath);
        logger->writeToLogMap(map, *sr.lppath);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}

const char *Mission::getAlgorithmName()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        return CNS_SP_ST_ASTAR;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK)
        return CNS_SP_ST_DIJK;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_BFS)
        return CNS_SP_ST_BFS;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_JP_SEARCH)
        return CNS_SP_ST_JP_SEARCH;
    else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_TH)
        return CNS_SP_ST_TH;
    else
        return "";
}
