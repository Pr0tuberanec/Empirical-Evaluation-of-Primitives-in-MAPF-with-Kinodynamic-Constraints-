#ifndef SIPP_H
#define SIPP_H
#include "gl_const.h"
#include "heap.h"
#include "ilogger.h"
#include "search.h"

#include <math.h>
#include <stdlib.h>

class SIPP : public Search
{
    public:
        SIPP();
        SIPP(double weight, int BT);
        ~SIPP();
        SearchResult startSearch();
    
    private:
        TimeNode getMin();

        std::vector<std::pair<int, int>> getFreeTimesteps(int nodeIdx);

        void initializeSearch();
        
        bool isGoalReachable(const TimeNode& curNode);
        void processNode(const TimeNode& curNode);                              

        bool isNodeInOpen(int nodeIdx, int start_t);
        bool isNodeInClose(int nodeIdx, int start_t);
        bool isValidSuccessor(int nodeIdx, int start_t);

        std::pair<int, int> searchIntrvl(int nodeIdx, std::pair<int, int> intrvl);
        bool checkMoveIntersection(int curNode_i, int curNode_j, int newNode_i,
                                   int newNode_j, int start_t, int end_t);
        std::vector<std::pair<int, int>> projectIntervals(
            Primitive edge, TimeNode startNode, Map* map);

        void getSuccessors(TimeNode curNode, std::vector<TimeNode>& successors);
        void updateSearchResult(Node* curNode, bool found,
            std::chrono::time_point<std::chrono::system_clock> start);

        bool stopCriterion();

        double hweight;      // weight of h-value
        bool breakingties;   // flag that sets the priority of nodes in addOpen function when their F-values are equal

        BinHeap open;
        std::unordered_map<int, std::unordered_map<int, TimeNode>> close;
};

#endif