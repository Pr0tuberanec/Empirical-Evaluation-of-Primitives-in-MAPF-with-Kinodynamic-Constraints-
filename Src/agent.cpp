#include "agent.h"

Agent::Agent(const Agent& other) {
    start_i = other.start_i;
    start_j = other.start_j;
    goal_i = other.goal_i;
    goal_j = other.goal_j;
}

Agent::~Agent() {
}