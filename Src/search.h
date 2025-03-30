#ifndef SEARCH_H
#define SEARCH_H

#include "environmentoptions.h"
#include "ilogger.h"
#include "searchresult.h"

#include <chrono>
#include <list>
#include <memory>

class Search
{
    public:
        Search();
        virtual ~Search(void);
        virtual SearchResult startSearch();

        void SetLogger(ILogger* inp_logger) { logger = inp_logger; };
        void SetMap(Map* inp_map) { map = inp_map; };
        void SetOptions(EnvironmentOptions* inp_options) { options = inp_options; };

    protected:
        virtual void updateSearchResult(Node* curNode, bool found,
                                        std::chrono::time_point<std::chrono::system_clock> start);
        virtual void makePrimaryPath(Node* curNode);
        virtual void makeSecondaryPath();

        // Три параметра с целью уменьшить число передаваемых аргументов в функции и улучшения структурности
        ILogger*                          logger;
        Map*                              map;
        EnvironmentOptions*               options;

        SearchResult                     sresult;
        std::list<std::shared_ptr<Node>> lppath, hppath;
        std::vector<std::vector<Node>>   obstacles_paths;
};

double computeHFromCellToCell(int i1, int j1, int i2, int j2, const EnvironmentOptions &options);
double computeCostToNeighbour(int i1, int j1, int i2, int j2, const EnvironmentOptions &options);

#endif
