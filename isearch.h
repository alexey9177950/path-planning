#ifndef ISEARCH_H
#define ISEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <set>
#include <list>
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <queue>

using NodeIter = std::set<Node>::iterator;

class ISearch
{
    public:
        ISearch();
        ~ISearch(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

    protected:
        //CODE HERE
        //Try to split class functionality to the methods that can be re-used in successors classes,
        //e.g. classes that realize A*, JPS, Theta* algorithms

        //Hint 1. You definetely need class variables for OPEN and CLOSE

        //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
        //for non-heuristic search methods like Dijkstra

        //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
        //will return it's sucessors, e.g. unordered list of nodes

        //Hint 4. It's a good idea to define a resetParent function that will be extensively used for Theta*
        //and for A*/Dijkstra/JPS will exhibit "default" behaviour

        //Hint 5. The last but not the least: working with OPEN and CLOSE is the core
        //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
        //Start with very simple (and ineffective) structures like list or vector and make it work first
        //and only then begin enhancement!



        virtual std::vector<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options);
        virtual void makePrimaryPath(Node curNode);//Makes path using back pointers
        virtual void makeSecondaryPath();//Makes another type of path(sections or points)
        Node create_node(int i, int j, double g, const Node* parent);
        double calc_dist(const Node& node_1,const Node& node_2);
        SearchResult sresult;
        std::list<Node> lppath, hppath;
        int metrictype;
        double hweight;//weight of h-value
        bool breakingties;//flag that sets the priority of nodes in addOpen function when their F-values is equal
        std::set<Node> open, close;
        std::set<NodeIter, NodeIterComp> open_queue;
        Node begin_node, goal_node;
};
#endif
