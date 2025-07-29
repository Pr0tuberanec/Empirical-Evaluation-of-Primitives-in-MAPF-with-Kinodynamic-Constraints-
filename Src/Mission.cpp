#include "Mission.h"

Mission::Mission(const char* input_filename) : filename(input_filename), logger(nullptr) {}

bool Mission::getMap() {
    return map.getMap(filename);
}

bool Mission::getConfig() {
    return config.getConfig(filename);
}

bool Mission::getTasks() {
    return agent_tasks.getTasks(filename, map.getHeight(), map.getWidth());
}

bool Mission::createLog() {
    logger = std::make_unique<JsonLogger>();
    return logger->getLog(filename);
}

void Mission::startSearch() {
    PBS planner(logger.get(), map, config, agent_tasks);
    search_results = planner.startSearch();
}

void Mission::printSearchResultsToConsole() {
    std::cout << "Path ";
    if (!search_results.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    if (search_results.pathfound) {
        std::cout << "cost=" << search_results.pathlength / 10 << std::endl;
    }
    std::cout << "time=" << search_results.time << std::endl;
}

void Mission::saveSearchResultsToLog() {
    logger->writeToLogSummary(search_results.pathlength, search_results.time);
    if (search_results.pathfound) {
        logger->writeToLogMap(map);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}

