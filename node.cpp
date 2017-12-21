#include <unordered_set>
#include <node.h>
#include <gl_const.h>

bool NodeComparator::operator()(const Node& node_1, const Node& node_2) const {
    if (node_1.F == node_2.F) {
        if (breakingties == CN_SP_BT_GMIN) {
            return node_1.g > node_2.g;
        } else {
            return node_1.g < node_2.g;
        }
    }
    return node_1.F > node_2.F;
}
