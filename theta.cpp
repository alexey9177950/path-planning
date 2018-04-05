#include "theta.h"
Theta::~Theta()
{
}

int sign(double x) {
    return round(x) / abs(round(x));
}

void Theta::resetParent(Node &u, const Node *v_pr, const Map &map) {
    const Node *v_pr_pr = v_pr->parent;
    if (v_pr_pr && this->lineOfSight(v_pr_pr->i, v_pr_pr->j, u.i, u.j, map, true)) {
        u.g = v_pr_pr->g + this->calc_eucl_dist(*v_pr_pr, u);
        u.parent = v_pr_pr;
    }
}

bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const Map &map, bool cutcorners)
{
    // Modified Bresenham's algorithm
    double di = i2 - i1, dj = j2 - j1;
    int pr_i = i1, pr_j = j1;
    if (fabs(di) > fabs(dj)) {
        int delta_i = sign(di);
        for (int i = i1; i != i2; i += delta_i) {
            int j = j1 + round(double(i - i1) * dj / di);
            if (map.getValue(i, j) != CN_GC_NOOBS) {
                return false;
            }
            if (!cutcorners && (map.getValue(pr_i, j) != CN_GC_NOOBS || map.getValue(i, pr_j))) {
                return false;
            }
            pr_i = i, pr_j = j;
        }
    } else {
        int delta_j = sign(dj);
        for (int j = j1; j != j2; j += delta_j) {
            int i = i1 + round(double(j - j1) * di / dj);
            if (map.getValue(i, j) != CN_GC_NOOBS) {
                return false;
            }
            if (!cutcorners && (map.getValue(pr_i, j) != CN_GC_NOOBS || map.getValue(i, pr_j))) {
                return false;
            }
            pr_i = i, pr_j = j;
        }
    }
    return true;
}

void Theta::makePrimaryPath(Node goal_node)
{
    // Using parent links to build primary path
    const Node *curNode = &goal_node;
    while (curNode != nullptr) {
        hppath.emplace_front(*curNode);
        curNode = curNode->parent;
    }
}

void Theta::makeSecondaryPath()
{
    std::list<Node>::iterator cur, prev;
    prev = hppath.begin();
    cur = ++hppath.begin();
    while (cur != hppath.end()) {
        // Add nodes on a segment between prev and cur
        int i1 = prev->i, j1 = prev->j;
        int i2 = cur->i, j2 = cur->j;
        double di = i2 - i1, dj = j2 - j1;
        if (fabs(di) > fabs(dj)) {
            int delta_i = sign(di);
            for (int i = i1; i != i2; i += delta_i) {
                int j = j1 + round(double(i - i1) * dj / di);
                Node new_node;
                new_node.i = i;
                new_node.j = j;
                lppath.push_back(new_node);
            }
        } else {
            int delta_j = sign(dj);
            for (int j = j1; j != j2; j += delta_j) {
                int i = i1 + round(double(j - j1) * di / dj);
                Node new_node;
                new_node.i = i;
                new_node.j = j;
                lppath.push_back(new_node);
            }
        }
        lppath.push_back(*cur);
        ++cur;
        ++prev;
    }
}
