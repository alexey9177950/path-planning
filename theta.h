#ifndef THETA_H
#define THETA_H
#include "astar.h"

class Theta: public Astar
{
    public:
        Theta(double hweight, bool breakingties):Astar(hweight, breakingties){}
        ~Theta(void);

    private:
        bool lineOfSight(int i1, int j1, int i2, int j2, const Map &map, bool cutcorners);
        void makePrimaryPath(Node) override;
        void makeSecondaryPath() override;
        void resetParent(Node &u, const Node *v_pr, const Map &map) override;
};


#endif // THETA_H
