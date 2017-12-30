#include "isearch.h"

ISearch::ISearch()
{
    open = std::priority_queue<Node, std::vector<Node>, NodeComparator>(NodeComparator(breakingties));
    sresult.pathfound = false;
    sresult.hppath = new std::list<Node>();
    sresult.lppath = new std::list<Node>();
}

ISearch::~ISearch(void) {}

double ISearch::calc_dist(const Node &n_1, const Node &n_2) {
    double dx = abs(n_1.j - n_2.j);
    double dy = abs(n_1.i - n_2.i);
    if (metrictype == CN_SP_MT_DIAG) {
        return std::max(dx, dy) + (CN_SQRT_TWO - 1.) * std::min(dx, dy);
    } else if (CN_SP_MT_MANH) {
        return dx + dy;
    } else if (CN_SP_MT_EUCL) {
        return sqrt(dx * dx + dy * dy);
    } else {
        // CN_SP_MT_CHEB
        return std::max(dx, dy);
    }
}

Node ISearch::create_node(int i, int j, double g, const Node *parent) {
    Node ans;
    ans.i = i;
    ans.j = j;
    ans.g = g;
    ans.H = calc_dist(ans, goal_node);
    ans.F = ans.g + hweight * ans.H;
    ans.parent = parent;
    return ans;
}

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    using namespace  std::chrono;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    begin_node = create_node(map.getStartI(), map.getStartJ(), 0, nullptr);
    goal_node = create_node(map.getGoalI(), map.getGoalJ(), 0, nullptr);

    open.push(begin_node);
    while (!open.empty()) {
        while (!open.empty() && close.find(open.top()) != close.end()) {
            open.pop();
        }
        if (open.empty()) {
            break;
        }
        Node v = open.top();
        open.pop();

        if (v == goal_node) {
            sresult.pathfound = true;
            sresult.pathlength = v.g;
            makePrimaryPath(v);
            makeSecondaryPath();
            break;
        }
        std::vector<Node> successors = findSuccessors(v, map, options);
        for (const Node& u : successors) {
            if (close.find(u) == close.end()) {
                open.push(u);
            }
        }
    }
    sresult.nodescreated =  open.size() + close.size();
    sresult.numberofsteps = close.size();
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    sresult.time = duration_cast<duration<double>>(t2 - t1).count();
    return sresult;
}

std::vector<Node> ISearch::findSuccessors(Node v, const Map &map, const EnvironmentOptions &options)
{
    const Node *v_ptr = &(*close.insert(v).first);
    std::vector<Node> successors;
    int dir_num;
    int di[] = {0,  1,  0, -1,  1,  1, -1, -1};
    int dj[] = {1,  0, -1,  0, -1,  1,  1, -1};
    if (options.allowdiagonal) {
        dir_num = 8;
    } else {
        dir_num = 4;
    }
    for (int dir = 0; dir < dir_num; ++dir) {
        int i_new = v.i + di[dir];
        int j_new = v.j + dj[dir];
        if (map.getValue(i_new, j_new) != CN_GC_NOOBS) {
            continue;
        }
        double new_g;
        if (dir < 4) {
            new_g = v.g + 1.;
        } else {
            new_g = v.g + CN_SQRT_TWO;
            int obst_n = map.CellIsObstacle(v.i, j_new) + map.CellIsObstacle(i_new, v.j);
            if ((obst_n == 2 && !options.allowsqueeze) ||  (obst_n >= 1 && !options.cutcorners)) {
                continue;
            }
        }
        successors.push_back(create_node(i_new, j_new, new_g, v_ptr));
    }
    return successors;
}

void ISearch::makePrimaryPath(Node goal_node)
{
    const Node *curNode = &goal_node;
    while (curNode != nullptr) {
        lppath.emplace_front(*curNode);
        curNode = curNode->parent;
    }
}


void ISearch::makeSecondaryPath()
{
    for (auto it = lppath.begin(); it != lppath.end(); ++it) {
        auto it_next = it, it_prev = it;
        ++it_next, --it_prev;
        if (it == lppath.begin() || it_next == lppath.end()) {
            hppath.push_back(*it);
        } else if (2 * it->i != it_prev->i + it_next->i || 2 * it->j != it_prev->j + it_next->j) {
            hppath.push_back(*it);
        }
    }
}
