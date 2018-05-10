#include "jp_search.h"

JP_Search::~JP_Search()
{
}

int sign(int x) {
    if (x < 0) {
        return -1;
    } else if (x > 0) {
        return 1;
    } else {
        return 0;
    }
}

std::vector<Node> JP_Search::findNeighbours(const Node &v, const Map& map, const EnvironmentOptions &options) {
    if (v.parent == nullptr) {
        return Astar::findSuccessors(v, map, options);
    }
    std::vector<Node> ans;
    int di = sign(v.i - v.parent->i), dj = sign(v.j - v.parent->j);
    if (di != 0 && dj != 0) { // diagonal move
        int cnt = 2;
        if (map.CellIsTraversable(v.i + di, v.j)) {
            ans.push_back(Node(v.i + di, v.j));
            --cnt;
            if (map.CellIsObstacle(v.i, v.j - dj) && map.CellIsTraversable(v.i + di, v.j - dj)) {
                ans.push_back(Node(v.i + di, v.j - dj));
            }
        }
        if (map.CellIsTraversable(v.i, v.j + dj)) {
            ans.push_back(Node(v.i, v.j + dj));
            --cnt;
            if (map.CellIsObstacle(v.i - di, v.j) && map.CellIsTraversable(v.i - di, v.j + dj)) {
                ans.push_back(Node(v.i - di, v.j + dj));
            }
        }
        if (cnt < 2 && map.CellIsTraversable(v.i + di, v.j + dj)) {
            ans.push_back(Node(v.i + di, v.j + dj));
        }
    } else if (di != 0) { // vertical move
        // natural neighbour:
        if (map.CellIsTraversable(v.i + di, v.j)) {
            ans.push_back(Node(v.i + di, v.j));
        }
        // forsed neighbours:
        for (int dj = -1; dj <= 1; dj += 2) {
            if (map.CellIsObstacle(v.i, v.j + dj) && map.CellIsTraversable(v.i + di, v.j + dj)) {
                ans.push_back(Node(v.i + di, v.j + dj));
            }
        }
    } else { // horizontal move
        // natural neighbour:
        if (map.CellIsTraversable(v.i, v.j + dj)) {
            ans.push_back(Node(v.i, v.j + dj));
        }
        // forsed neighbours:
        for (int di = -1; di <= 1; di += 2) {
            if (map.CellIsObstacle(v.i + di, v.j) && map.CellIsTraversable(v.i + di, v.j + dj)) {
                ans.push_back(Node(v.i + di, v.j + dj));
            }
        }
    }
    return ans;
}

// returns Node with i = -1 if nothing was found
Node JP_Search::jump(Node v, Node v_pr, const Map &map, const EnvironmentOptions &opt) {
    if (map.CellIsObstacle(v.i, v.j)) {
        v.i = -1;
        return v;
    }
    if (v == goal_node) {
        return v;
    }
    int di = v.i - v_pr.i, dj = v.j - v_pr.j;
    if (di != 0 && dj != 0) {
        // diagonal case
        // check forsed neighbours
        if (map.CellIsObstacle(v.i - di, v.j) && map.CellIsTraversable(v.i - di, v.j + dj)) {
            return v;
        }
        if (map.CellIsObstacle(v.i, v.j - dj) && map.CellIsTraversable(v.i + di, v.j - dj)) {
            return v;
        }
        // check vertical and horizontal jump
        if (jump(Node(v.i, v.j + dj), v, map, opt).i != -1 || jump(Node(v.i + di, v.j), v, map, opt).i != -1) {
            return v;
        }
        return jump(Node(v.i + di, v.j + dj), v, map, opt);
    } else if (di != 0) {
        // vertical case
        for (int dj = -1; dj <= 1; dj += 2) {
            if (map.CellIsObstacle(v.i, v.j + dj) && map.CellIsTraversable(v.i + di, v.j + dj)) {
                return v;
            }
        }
        return jump(Node(v.i + di, v.j), v, map, opt);
    } else {
        // horizontal case
        for (int di = -1; di <= 1; di += 2) {
            if (map.CellIsObstacle(v.i + di, v.j) && map.CellIsTraversable(v.i + di, v.j + dj)) {
                return v;
            }
        }
        return jump(Node(v.i, v.j + dj), v, map, opt);
    }
}

std::vector<Node> JP_Search::findSuccessors(Node v, const Map &map, const EnvironmentOptions &options)
{
    // implementation with allowdiagonal = true, cutcorners = true, allowsqueze = false
    std::vector<Node> neighbours = findNeighbours(v, map, options);
    std::vector<Node> successors;
    for (Node u : neighbours) {
        u = jump(u, v, map, options);
        if (u.i != -1) {
            u.parent = &(*close.find(v));
            u.H = calc_dist(u, goal_node);
            u.g = v.g + calc_eucl_dist(v, u);
            u.F = u.g + u.H * hweight;
            successors.push_back(u);
        }
    }
    return successors;
}

void JP_Search::makePrimaryPath(Node endNode)
{
    std::list<Node> hppath_0;
    // Using parent links to build primary path
    const Node *curNode = &endNode;
    while (curNode != nullptr) {
        hppath_0.emplace_front(*curNode);
        curNode = curNode->parent;
    }
    // delete nodes on a segment between previous and next
    for (auto it = hppath_0.begin(); it != hppath_0.end(); ++it) {
        auto it_next = it, it_prev = it;
        ++it_next;
        if (it != hppath_0.begin()) {
            --it_prev;
        }
        if (it == hppath_0.begin() || it_next == hppath_0.end()) {
            hppath.push_back(*it);
        } else {
            int di1 = sign(it->i - it_prev->i), dj1 = sign(it->j - it_prev->j);
            int di2 = sign(it_next->i - it->i), dj2 = sign(it_next->j - it->j);
            if (di1 != di2 ||  dj1 != dj2) {
                hppath.push_back(*it);
            }
        }
    }
}

void JP_Search::makeSecondaryPath()
{
    std::list<Node>::iterator cur, prev;
    prev = hppath.begin();
    cur = ++hppath.begin();
    while (cur != hppath.end()) {
        // Add nodes on a segment between prev and cur
        int i1 = prev->i, j1 = prev->j;
        int i2 = cur->i, j2 = cur->j;
        int di = sign(i2 - i1), dj = sign(j2 - j1);
        int i = i1, j = j1;
        while (i != i2 || j != j2) {
            lppath.push_back(Node(i, j));
            i += di;
            j += dj;
        }
        lppath.push_back(*cur);
        ++cur;
        ++prev;
    }
}
