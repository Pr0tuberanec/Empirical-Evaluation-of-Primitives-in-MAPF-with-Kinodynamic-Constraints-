#ifndef JSONLOGGER_H
#define JSONLOGGER_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "json.hpp"

#include "Map.h"
#include "Node.h"

class JsonLogger {
public:

    JsonLogger() = default;
    ~JsonLogger() = default;

    bool getLog(const char* filename);
    void saveLog();

    void writeToLogSummary(float length, double time);
    void writeToLogMap(const Map& map);
    void writeToLogAgentPaths(const std::vector<std::shared_ptr<std::vector<Node>>>& agent_paths);
    void writeToLogNotFound();

private:
    std::string log_filename;
    nlohmann::ordered_json parsed_json;
};

#endif
