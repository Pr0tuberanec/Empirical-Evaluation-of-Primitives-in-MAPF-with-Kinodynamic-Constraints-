#ifndef TASKS_H
#define TASKS_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "json.hpp"

#include "Agent.h"
#include "Constants.h"

class Tasks {
public:

    Tasks() = default;

    bool getTasks(const char* filename, int map_height, int map_width);

    std::vector<Agent>& getAgentsNoConstRef();

private:
    std::vector<Agent> agents;
};

#endif
