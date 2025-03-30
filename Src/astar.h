#ifndef ASTAR_H
#define ASTAR_H

#include "ilogger.h"
#include "search.h"
#include "searchresult.h"

#include <queue>

class Astar : public Search
{
    public:
        Astar();
        Astar(double weight, int BT);
        ~Astar();

        SearchResult startSearch();

    private:
        void getSuccessors(const Node& curNode);
        void updateSearchResult(Node* curNode, bool found,
            std::chrono::time_point<std::chrono::system_clock> start);

        double hweight;      // weight of h-value
        bool breakingties;   // flag that sets the priority of nodes in addOpen function when their F-values are equal

        std::unordered_map<int, Node> close;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
};

#endif