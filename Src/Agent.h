#ifndef AGENT_H
#define	AGENT_H

#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "Constants.h"
#include "Map.h"
#include "Node.h"

class Agent {
public:

    Agent(int si, int sj, int gi, int gj);

    void initializeReservationTable(const Map& map, int T_max);
    void buildReservationTable(const std::vector<std::shared_ptr<std::vector<Node>>>& paths,
                               const std::vector<int>& indices);
    void normalizeReservationTable();
    void clrReservationTable(const Map& map, int T_max);

    int getStartI() const;
    int getStartJ() const;
    int getGoalI() const;
    int getGoalJ() const;
    const std::vector<std::vector<std::set<std::pair<int, int>>>>& getRsrvTbl() const;

private:

    int start_i, start_j;
    int goal_i, goal_j; 
    std::vector<std::vector<std::set<std::pair<int, int>>>> rsrv_tbl;

    void unionIntervals(std::set<std::pair<int, int>>& intervals) const;
};

#endif
