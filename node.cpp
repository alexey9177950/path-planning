#include "node.h"
#include "gl_const.h"

bool NodeIterComp::operator()(NodeIter node_1, NodeIter node_2) const {
    if (node_1->F == node_2->F) {
        if (breakingties == CN_SP_BT_GMIN) {
            if (node_1->g == node_2->g) {
                return *node_1 < *node_2;
            } else {
                return node_1->g < node_2->g;
            }
        } else {
            if (node_1->g == node_2->g) {
                return *node_1 < *node_2;
            } else {
                return node_1->g > node_2->g;
            }
        }
    }
    return node_1->F < node_2->F;
}
