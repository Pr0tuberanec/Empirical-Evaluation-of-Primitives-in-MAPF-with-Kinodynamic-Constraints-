#ifndef CONFIG_H
#define	CONFIG_H

#include <cmath> 
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "json.hpp"

#include "Constants.h"

class Config {
public:

    Config();
    ~Config();

    bool getConfig(const char *filename);
    double computeHFromCellToCell(int i1, int j1, int i2, int j2) const;

    int getSearchParam(int index) const;

private:
    std::shared_ptr<std::vector<int>> search_params;
};  

#endif

