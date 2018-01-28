#include "theta.h"
Theta::~Theta()
{
}

std::vector<Node> Theta::findSuccessors(Node v, const Map &map, const EnvironmentOptions &options) {
    std::cout << "f succ in thetta" << std::endl;
    const Node *v_ptr = &(*close.insert(v).first);
    const Node *vp_ptr = v_ptr->parent;
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
        if (dir >= 4) {
            int obst_n = map.CellIsObstacle(v.i, j_new) + map.CellIsObstacle(i_new, v.j);
            if (((obst_n == 2 && !options.allowsqueeze) ||  (obst_n >= 1 && !options.cutcorners))) {
                continue;
            }
        }
        Node u = create_node(i_new, j_new, 0, nullptr);
        if (vp_ptr && this->lineOfSight(vp_ptr->i, vp_ptr->j, u.i, u.j, map, options.cutcorners)) {
            u.g = vp_ptr->g + this->calc_dist(*vp_ptr, u);
            u.parent = vp_ptr;
        } else {
            u.g = v_ptr->g + this->calc_dist(*v_ptr, u);
            u.parent = v_ptr;
        }
        successors.push_back(u);
    }
    return successors;
}


bool Theta::lineOfSight(int i1, int j1, int i2, int j2, const Map &map, bool cutcorners)
{
    double di= abs(i1 - i2), dj = abs(j1 - j2);
    if (fabs(di) > fabs(dj)) {
        for (int i = std::min(i1, i2); i < std::max(i1, i2); ++i) {
            int j = j1 + round(double(i - i1) * dj / di);
            if (map.getValue(i, j) != CN_GC_NOOBS) {
                return false;
            }
        }
    } else {
        for (int j = std::min(j1, j2); j < std::max(j1, j2); ++j) {
            int i = i1 + round(double(j - j1) * di / dj);
            if (map.getValue(i, j) != CN_GC_NOOBS) {
                return false;
            }
        }
    }
    return true;
}

void Theta::makePrimaryPath(Node goal_node)
{
    std::cout << "PRIMARY" << std::endl;
    const Node *curNode = &goal_node;
    while (curNode != nullptr) {
        hppath.emplace_front(*curNode);
        curNode = curNode->parent;
    }
}


void Theta::makeSecondaryPath()
{
    std::cout << "SECONDARY" << std::endl;
    lppath = hppath;
    //need to implement
}
