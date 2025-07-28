#ifndef MISSION_H
#define	MISSION_H

#include <filesystem>
#include <memory>

#include "Config.h"
#include "Parse.h"
#include "JsonLogger.h"
#include "Map.h"
#include "Pbs.h"
#include "Searchresult.h"

class Mission {
public:

    Mission (const char* filename);
    ~Mission() = default;

    bool getMap();
    bool getConfig();
    bool getTasks();

    bool createLog();

    void startSearch();

    void printSearchResultsToConsole();
    void saveSearchResultsToLog();
    
private:
    const char*                     filename;
    Map                             map;
    Config                          config;
    Tasks                           agent_tasks;
    SearchResult                    search_results;
    std::unique_ptr<JsonLogger>     logger;
};

#endif

