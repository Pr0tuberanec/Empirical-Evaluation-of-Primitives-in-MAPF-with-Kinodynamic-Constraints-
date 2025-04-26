#ifndef AGENT_H
#define	AGENT_H
#include <iostream>
#include "gl_const.h"
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

class Agent
{
    public:
        int start_i, start_j;
        int goal_i, goal_j;

    public:
        Agent();
        Agent(int sy, int sx, int fy, int fx)
            : start_i(sy), start_j(sx), goal_i(fy), goal_j(fx) {}
        Agent(const Agent& other);
        ~Agent();

        bool getAgent(const char *FileName);
};

#endif