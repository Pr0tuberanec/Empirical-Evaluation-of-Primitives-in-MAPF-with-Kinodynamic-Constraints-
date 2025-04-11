#include "config.h"
#include "gl_const.h"
#include "tinyxml2.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <math.h>
#include "json.hpp"
#include <fstream>
using json = nlohmann::json;

Config::Config()
{
    LogParams = nullptr;
    SearchParams = nullptr;
}

Config::~Config()
{
    if (SearchParams) delete[] SearchParams;
    if (LogParams) delete[] LogParams;
}

bool Config::getConfig(const char *FileName)
{
    std::string value;
    std::ifstream file(FileName);
    if (!json::accept(file))
    {
        std::cout << "Error opening JSON file!" << std::endl;
        return false;
    }
    json objJson = json::parse(std::ifstream(FileName));

    json root;
    if (!objJson.contains(CNS_TAG_ROOT)) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' element found in JSON file!" << std::endl;
        return false;
    }
    root = objJson[CNS_TAG_ROOT];

    json algorithm;
    if (!root.contains(CNS_TAG_ALG)) {
        std::cout << "Error! No '" << CNS_TAG_ALG << "' tag found in JSON file!" << std::endl;
        return false;
    }
    algorithm = root[CNS_TAG_ALG];

    json element;
    if (!algorithm.contains(CNS_TAG_ST)) {
        std::cout << "Error! No '" << CNS_TAG_ST << "' tag found in JSON file!" << std::endl;
        return false;
    }
    element = algorithm[CNS_TAG_ST];
    if (!element.is_string())
        value = to_string(element);
    else
        value = element;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);

    if (value == CNS_SP_ST_ASTAR || value == CNS_SP_ST_SIPP || value == CNS_SP_ST_PBS) {
        N = 10;
        SearchParams = new double[N];
        SearchParams[CN_SP_ST] = CN_SP_ST_ASTAR;
        if (value == CNS_SP_ST_SIPP)
            SearchParams[CN_SP_ST] = CN_SP_ST_SIPP;
        else if (value == CNS_SP_ST_PBS)
            SearchParams[CN_SP_ST] = CN_SP_ST_PBS;
        if (!algorithm.contains(CNS_TAG_HW)) {
            std::cout << "Warning! No '" << CNS_TAG_HW << "' tag found in algorithm section." << std::endl;
            std::cout << "Value of '" << CNS_TAG_HW << "' was defined to 1." << std::endl;
            SearchParams[CN_SP_HW] = 1;
        } else {
            element = algorithm[CNS_TAG_HW];
            if (element.is_number()) {
                SearchParams[CN_SP_HW] = static_cast<double>(element);

                if (SearchParams[CN_SP_HW] < 1) {
                    std::cout << "Warning! Value of '" << CNS_TAG_HW << "' tag is not correctly specified. Should be >= 1."
                              << std::endl;
                    std::cout << "Value of '" << CNS_TAG_HW << "' was defined to 1." << std::endl;
                    SearchParams[CN_SP_HW] = 1;
                }
            } else {
                std::cout << "Warning! Value of '" << CNS_TAG_HW << "' tag is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_HW << "' was defined to 1." << std::endl;
                SearchParams[CN_SP_HW] = 1;
            }
        }

        if (!algorithm.contains(CNS_TAG_MT)) {
            std::cout << "Warning! No '" << CNS_TAG_MT << "' tag found in JSON file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_MT << "' was defined to 'euclidean'." << std::endl;
            SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
        } else {
            element = algorithm[CNS_TAG_MT];
            if (!element.is_string())
                value = to_string(element);
            else
                value = element;
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if (value == CNS_SP_MT_MANH) SearchParams[CN_SP_MT] = CN_SP_MT_MANH;
            else if (value == CNS_SP_MT_EUCL) SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
            else if (value == CNS_SP_MT_DIAG) SearchParams[CN_SP_MT] = CN_SP_MT_DIAG;
            else if (value == CNS_SP_MT_CHEB) SearchParams[CN_SP_MT] = CN_SP_MT_CHEB;
            else {
                std::cout << "Warning! Value of'" << CNS_TAG_MT << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_MT << "' was defined to 'euclidean'" << std::endl;
                SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
            }
            // if (SearchParams[CN_SP_ST] == CN_SP_ST_TH && SearchParams[CN_SP_MT] != CN_SP_MT_EUCL) {
            //     std::cout << "Warning! This type of metric is not admissible for Theta*!" << std::endl;
            // }
        }

        if (!algorithm.contains(CNS_TAG_BT)) {
            std::cout << "Warning! No '" << CNS_TAG_BT << "' tag found in JSON file" << std::endl;
            std::cout << "Value of '" << CNS_TAG_BT << "' was defined to 'g-max'" << std::endl;
            SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
        } else {
            element = algorithm[CNS_TAG_BT];
            if (!element.is_string())
                value = to_string(element);
            else
                value = element;
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if (value == CNS_SP_BT_GMIN) SearchParams[CN_SP_BT] = CN_SP_BT_GMIN;
            else if (value == CNS_SP_BT_GMAX) SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
            else {
                std::cout << "Warning! Value of '" << CNS_TAG_BT << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_BT << "' was defined to 'g-max'" << std::endl;
                SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
            }
        }

        if (!algorithm.contains(CNS_T_MAX)) {
            std::cout << "Warning! No '" << CNS_T_MAX << "' tag found in JSON file" << std::endl;
            std::cout << "Value of '" << CNS_T_MAX << "' was defined to 100" << std::endl;
            SearchParams[CN_T_MAX] = 100;
        } else {
            element = algorithm[CNS_T_MAX];
            SearchParams[CN_T_MAX] = static_cast<double>(element);
        }

        if (!algorithm.contains(CNS_SEED)) {
            std::cout << "Warning! No '" << CNS_SEED << "' tag found in JSON file" << std::endl;
            std::cout << "Value of '" << CNS_SEED << "' was defined to 0" << std::endl;
            SearchParams[CN_SEED] = 0;
        } else {
            element = algorithm[CNS_SEED];
            SearchParams[CN_SEED] = static_cast<double>(element);
        }

        if (!algorithm.contains(CNS_NUM_OBS)) {
            std::cout << "Warning! No '" << CNS_NUM_OBS << "' tag found in JSON file" << std::endl;
            std::cout << "Value of '" << CNS_NUM_OBS << "' was defined to 0" << std::endl;
            SearchParams[CN_NUM_OBS] = 0;
        } else {
            element = algorithm[CNS_NUM_OBS];
            SearchParams[CN_NUM_OBS] = static_cast<double>(element);
        }

    } else {
        std::cout << "Error! Value of '" << CNS_TAG_ST << "' tag (algorithm name) is not correctly specified."
                  << std::endl;
        std::cout << "Supported algorithm's names are: '" << CNS_SP_ST_ASTAR << "', '" << CNS_SP_ST_SIPP << "', '"
                  << CNS_SP_ST_PBS << "'." << std::endl;
        return false;
    }


    if (!algorithm.contains(CNS_TAG_AD)) {
        std::cout << "Warning! No '" << CNS_TAG_AD << "' element found in JSON file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_AD << "' was defined to default - true" << std::endl;
        SearchParams[CN_SP_AD] = 1;
    } else {
        element = algorithm[CNS_TAG_AD];
        std::string check;
        if (!element.is_string())
            check = to_string(element);
        else
            check = element;
        if (check != "1" && check != "true" && check != "0" && check != "false") {
            std::cout << "Warning! Value of '" << CNS_TAG_AD << "' is not correctly specified." << std::endl;
            std::cout << "Value of '" << CNS_TAG_AD << "' was defined to default - true " << std::endl;
            SearchParams[CN_SP_AD] = 1;
        }
        else if (check == "1" || check == "true")
            SearchParams[CN_SP_AD] = 1;
        else
            SearchParams[CN_SP_AD] = 0;
    }

    if (SearchParams[CN_SP_AD] == 0) {
        SearchParams[CN_SP_CC] = 0;
        SearchParams[CN_SP_AS] = 0;
    } else {
        if (!algorithm.contains(CNS_TAG_CC)) {
            std::cout << "Warning! No '" << CNS_TAG_CC << "' element found in JSON file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_CC << "' was defined to default - false" << std::endl;
            SearchParams[CN_SP_CC] = 0;
        } else {
            element = algorithm[CNS_TAG_CC];
            std::string check;
            if (!element.is_string())
                check = to_string(element);
            else
                check = element;
            if (check != "1" && check != "true" && check != "0" && check != "false") {
                std::cout << "Warning! Value of '" << CNS_TAG_CC << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_CC << "' was defined to default - false" << std::endl;
                SearchParams[CN_SP_CC] = 0;
            } else {
                if (check == "1" || check == "true")
                    SearchParams[CN_SP_CC] = 1;
                else
                    SearchParams[CN_SP_CC] = 0;
            }
        }
        if (SearchParams[CN_SP_CC] == 0) {
            SearchParams[CN_SP_AS] = 0;
        } else {
            if (!algorithm.contains(CNS_TAG_AS)) {
                std::cout << "Warning! No '" << CNS_TAG_AS << "' element found in JSON file." << std::endl;
                std::cout << "Value of '" << CNS_TAG_AS << "' was defined to default - false." << std::endl;
                SearchParams[CN_SP_AS] = 0;
            } else {
                element = algorithm[CNS_TAG_AS];
                std::string check;
                if (!element.is_string())
                    check = to_string(element);
                else
                    check = element;
                if (check != "1" && check != "true" && check != "0" && check != "false") {
                    std::cout << "Warning! Value of '" << CNS_TAG_AS << "' is not correctly specified." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_AS << "' was defined to default - false." << std::endl;
                    SearchParams[CN_SP_AS] = 0;
                } else {
                    if (check == "1" || check == "true")
                        SearchParams[CN_SP_AS] = 1;
                    else
                        SearchParams[CN_SP_AS] = 0;
                }
            }
        }
    }

    LogParams = new std::string[3];
    LogParams[CN_LP_PATH] = "";
    LogParams[CN_LP_NAME] = "";

    if (!root.contains(CNS_TAG_OPT)) {
        std::cout << "Warning! No '" << CNS_TAG_OPT << "' tag found in JSON file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_LOGLVL << "' tag was defined to 'short log' (1)." << std::endl;
        LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
    } else {
        json options = root[CNS_TAG_OPT];
        if (!options.contains(CNS_TAG_LOGLVL)) {
            std::cout << "Warning! No '" << CNS_TAG_LOGLVL << "' tag found in JSON file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGLVL << "' tag was defined to 'short log' (1)." << std::endl;
            LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
        } else {
            element = options[CNS_TAG_LOGLVL];
            if (!element.is_string())
                value = to_string(element);
            else
                value = element;
            //std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if (value == CN_LP_LEVEL_NOPE_WORD || value == CN_LP_LEVEL_NOPE_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_NOPE_WORD;
            else if (value == CN_LP_LEVEL_TINY_WORD || value == CN_LP_LEVEL_TINY_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_TINY_WORD;
            else if (value == CN_LP_LEVEL_SHORT_WORD || value == CN_LP_LEVEL_SHORT_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
            else if (value == CN_LP_LEVEL_MEDIUM_WORD || value == CN_LP_LEVEL_MEDIUM_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_MEDIUM_WORD;
            else if (value == CN_LP_LEVEL_FULL_WORD || value == CN_LP_LEVEL_FULL_VALUE)
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_FULL_WORD;
            else {
                std::cout << "'" << CNS_TAG_LOGLVL << "' is not correctly specified" << std::endl;
                std::cout << "Value of '" << CNS_TAG_LOGLVL << "' tag was defined to 'short log' (1)." << std::endl;
                LogParams[CN_LP_LEVEL] = CN_LP_LEVEL_SHORT_WORD;
            }
            std::cout << LogParams[CN_LP_LEVEL] << std::endl;
        }


        if (!options.contains(CNS_TAG_LOGPATH)) {
            std::cout << "Warning! No '" << CNS_TAG_LOGPATH << "' tag found in JSON file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGPATH << "' tag was defined to 'current directory'." << std::endl;
        } else {
            element = options[CNS_TAG_LOGPATH];
            if (!element.is_string()) {
                std::cout << "Warning! Value of '" << CNS_TAG_LOGPATH << "' tag is missing!" << std::endl;
                std::cout << "Value of '" << CNS_TAG_LOGPATH << "' tag was defined to 'current directory'." << std::endl;
            } else {
                LogParams[CN_LP_PATH] = element;
            }
        }


        if (!options.contains(CNS_TAG_LOGFN)) {
            std::cout << "Warning! No '" << CNS_TAG_LOGFN << "' tag found in JSON file!" << std::endl;
            std::cout << "Value of '" << CNS_TAG_LOGFN
                      << "' tag was defined to default (original filename +'_log' + original file extension."
                      << std::endl;
        } else {
            element = options[CNS_TAG_LOGFN];
            if (!element.is_string()) {
                std::cout << "Warning! Value of '" << CNS_TAG_LOGFN << "' tag is missing." << std::endl;
                std::cout << "Value of '" << CNS_TAG_LOGFN
                          << "' tag was defined to default (original filename +'_log' + original file extension."
                          << std::endl;
            } else
                LogParams[CN_LP_NAME] = element;
        }
    }
    return true;
}