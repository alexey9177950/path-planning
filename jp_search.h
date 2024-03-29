#ifndef JP_SEARCH_H
#define JP_SEARCH_H
#include "astar.h"

class JP_Search:public Astar
{
public:
    JP_Search(float hweight, bool breakingties):Astar(hweight, breakingties){}
    ~JP_Search();

private:
    std::vector<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options) override;
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();

    std::vector<Node> findNeighbours(const Node &v, const Map& map, const EnvironmentOptions &options);
    Node jump(Node v, Node v_pr, const Map &map, const EnvironmentOptions &opt);
};

#endif // JP_SEARCH_H
