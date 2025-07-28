#ifndef PARSE_H
#define PARSE_H

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include <json.hpp>

namespace fs = std::filesystem;

fs::path buildJsonFromMapScen(const fs::path& map, const fs::path& scen,
                              const fs::path& config,
                              int num_agents);

fs::path prepareTaskFromArgs(int argc, char* argv[], int& num_agents);
#endif