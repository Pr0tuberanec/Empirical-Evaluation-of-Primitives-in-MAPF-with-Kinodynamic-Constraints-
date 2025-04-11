#include "mission.h"
#include <iostream>

Mission::Mission()
{
    logger = nullptr;
    fileName = nullptr;
}

Mission::Mission(const char *FileName)
{
    fileName = FileName;
    logger = nullptr;
}

Mission::~Mission()
{
    if (logger)
        delete logger;
    if (search)
        delete search;
}

bool Mission::getMap()
{
    return map.getMap(fileName);
}

bool Mission::getConfig()
{
    return config.getConfig(fileName);
}

bool Mission::createLog()
{
    if (logger != nullptr) delete logger;
    logger = new JsonLogger(config.LogParams[CN_LP_LEVEL]);
    return logger->getLog(fileName, config.LogParams);
}

void Mission::createEnvironmentOptions()
{
    options.cutcorners = config.SearchParams[CN_SP_CC];
    options.allowsqueeze = config.SearchParams[CN_SP_AS];
    options.allowdiagonal = config.SearchParams[CN_SP_AD];
    options.metrictype = config.SearchParams[CN_SP_MT];

    T_max = config.SearchParams[CN_T_MAX];
    seed = config.SearchParams[CN_SEED];
    number_of_obstacles = config.SearchParams[CN_NUM_OBS];
}

void Mission::createSearch()
{
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR)
        search = new Astar(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_SIPP)
        search = new SIPP(config.SearchParams[CN_SP_HW], config.SearchParams[CN_SP_BT]);
}

void Mission::startSearch()
{
    search->SetLogger(logger);
    search->SetMap(&map);
    search->SetOptions(&options);

    dyn_obs_creator.createDynObstacles(logger, &map, &options, seed, number_of_obstacles);
    dyn_obs_creator.timeIntervalBuilding(T_max, map);
    search->SetTMax(T_max);
    search->SetPtrFreeTimestepsTable(dyn_obs_creator.GetPtrFreeTimestepsTable());
    search->SetPtrObstaclesTable(dyn_obs_creator.GetPtrObsTable());
    search->SetPtrObstaclesPaths(dyn_obs_creator.GetPtrObstaclesPaths());
    sr = search->startSearch();
}

void Mission::printSearchResultsToConsole()
{
    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;
    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;
    if (sr.pathfound) {
        std::cout << "pathlength=" << sr.pathlength << std::endl;
        std::cout << "pathlength_scaled=" << sr.pathlength * map.cellSize << std::endl;
    }
    std::cout << "time=" << sr.time << std::endl;
}

void Mission::saveSearchResultsToLog()
{
    logger->writeToLogSummary(sr.numberofsteps, sr.nodescreated, sr.pathlength, sr.time, map.cellSize);
    if (sr.pathfound) {
        logger->writeToLogPath(*sr.lppath);
        logger->writeToLogHPpath(*sr.hppath);
        logger->writeToLogMap(map, *sr.lppath);
    } else
        logger->writeToLogNotFound();
    logger->saveLog();
}

SearchResult Mission::getSearchResult()
{
    return sr;
}

