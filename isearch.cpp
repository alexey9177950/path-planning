#include "isearch.h"

ISearch::ISearch()
{
    open_queue = std::set<NodeIter, NodeIterComp>(NodeIterComp(breakingties));
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
    } else if (metrictype == CN_SP_MT_MANH) {
        return dx + dy;
    } else if (metrictype == CN_SP_MT_EUCL) {
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
    metrictype = options.metrictype;
    begin_node = create_node(map.getStartI(), map.getStartJ(), 0, nullptr);
    goal_node = create_node(map.getGoalI(), map.getGoalJ(), 0, nullptr);

    auto begin_iter = open.insert(begin_node).first;
    open_queue.insert(begin_iter);
    while (!open.empty()) {
        while (!open.empty() && close.find(**open_queue.begin()) != close.end()) {
            open.erase(*open_queue.begin());
            open_queue.erase(open_queue.begin());
        }
        if (open.empty()) {
            break;
        }
        Node cur_node = **open_queue.begin();
        open.erase(*open_queue.begin());
        open_queue.erase(open_queue.begin());
        close.insert(cur_node);

        if (cur_node == goal_node) {
            sresult.pathfound = true;
            sresult.pathlength = cur_node.g;
            makePrimaryPath(cur_node);
            makeSecondaryPath();
            break;
        }
        std::vector<Node> successors = findSuccessors(cur_node, map, options);
        for (const Node& succ_node : successors) {
            if (close.find(succ_node) == close.end()) {
                auto node_iter = open.find(succ_node);
                if (node_iter == open.end()) {
                    auto ans = open.insert(succ_node);
                    open_queue.insert(ans.first);
                } else if (node_iter->F > succ_node.F) {
                    open_queue.erase(node_iter);
                    open.erase(*node_iter);
                    auto ans = open.insert(cur_node);
                    open_queue.insert(ans.first);
                }
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
