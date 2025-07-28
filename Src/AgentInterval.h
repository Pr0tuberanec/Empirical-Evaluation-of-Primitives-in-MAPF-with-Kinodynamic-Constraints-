#ifndef AGENT_INTERVAL_H
#define AGENT_INTERVAL_H

#include <utility>

struct AgentInterval {
    std::pair<int, int> interval;
    int agent_id;
    
    bool operator<(const AgentInterval& other) const {
        if (interval.first != other.interval.first) {
            return interval.first < other.interval.first;
        }

        if (interval.second != other.interval.second) {
            return interval.second < other.interval.second;
        }

        return agent_id < other.agent_id; 
    }
};

#endif