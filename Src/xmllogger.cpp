#include "xmllogger.h"
#include <vector>
#include "node.h"
#include <iostream>
#include "json.hpp"
#include <fstream>
#include <string>

bool JsonLogger::getLog(const char *FileName, const std::string *LogParams)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD) return true;

    if (LogParams[CN_LP_PATH] == "" && LogParams[CN_LP_NAME] == "") {
        std::string str;
        str.append(FileName);
        size_t found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    } else if (LogParams[CN_LP_PATH] == "") {
        LogFileName.append(FileName);
        std::string::iterator it = LogFileName.end();
        while (*it != '\\')
            it--;
        ++it;
        LogFileName.erase(it, LogFileName.end());
        LogFileName.append(LogParams[CN_LP_NAME]);
    } else if (LogParams[CN_LP_NAME] == "") {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        std::string lfn;
        lfn.append(FileName);
        size_t found = lfn.find_last_of("\\");
        std::string str = lfn.substr(found);
        found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    } else {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        LogFileName.append(LogParams[CN_LP_NAME]);
    }

    objJson[CNS_TAG_ROOT] = nlohmann::ordered_json::object_t{};
    nlohmann::ordered_json log;
    if (loglevel != CN_LP_LEVEL_NOPE_WORD) {
        log[CNS_TAG_MAPFN] = FileName;

        log[CNS_TAG_SUM] = nlohmann::ordered_json::object_t{};

        log[CNS_TAG_PATH] = nlohmann::json::array_t();

        log[CNS_TAG_LPLEVEL] = nlohmann::ordered_json::object_t{};

        log[CNS_TAG_HPLEVEL] = nlohmann::ordered_json::object_t{};
    }

    if (loglevel == CN_LP_LEVEL_FULL_WORD || loglevel == CN_LP_LEVEL_MEDIUM_WORD)
        log[CNS_TAG_LOWLEVEL] = nlohmann::ordered_json::object_t{};

    objJson[CNS_TAG_ROOT][CNS_TAG_LOG] = log;
    return true;
}

void JsonLogger::saveLog()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;
    std::ofstream out(LogFileName.c_str());
    if (out.is_open()) {
        out << objJson.dump(4); // dump(4) для отступа в 4 пробела
        out.close();
        std::cout << "JSON успешно сохранен в " << "output.json" << std::endl;
    } else {
        std::cout << LogFileName << "<--------------" << std::endl;
        std::cerr << "Не удалось открыть файл для записи!" << std::endl;
    }
}

void JsonLogger::writeToLogMap(const Map& map, const std::vector<Node>& path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD)
        return;

    nlohmann::ordered_json mapTag;

    int iterate = 0;
    bool inPath;
    for (int i = 0; i < map.height; ++i) {
        std::string str = "[";
        for (int j = 0; j < map.width; ++j) {
            inPath = false;
            for(std::vector<Node>::const_iterator it = path.begin(); it != path.end(); it++)
                if(it->i == i && it->j == j) {
                    inPath = true;
                    break;
                }
            if (!inPath) {
                str += std::to_string(map.getValue(i,j));
            }
            else {
                str += CNS_OTHER_PATHSELECTION;
            }
            str += CNS_OTHER_MATRIXSEPARATOR;
        }
        str.pop_back();
        str.push_back(']');
        mapTag[iterate] = str;
        iterate++;
    }
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_WIDTH] = map.width;
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_HEIGHT] = map.height;
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_START] = "start_x = " + std::to_string(map.start_j) + ", " + "start_y = " + std::to_string(map.start_i);
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_FINISH] = "finish_x = " + std::to_string(map.goal_j) + ", " + "finish_y = " + std::to_string(map.goal_i);
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_PATH] = mapTag;

}
/*
void JsonLogger::writeToLogOpenClose(const std::vector<std::list<Node> > &open,
                                    const std::unordered_map<int, Node> &close,
                                    bool last)
{
    if (loglevel != CN_LP_LEVEL_FULL_WORD && !(loglevel == CN_LP_LEVEL_MEDIUM_WORD && last))
        return;

    XMLElement *element = doc.NewElement(CNS_TAG_STEP);
    XMLElement *child = 0;
    XMLElement *lowlevel = doc.FirstChildElement(CNS_TAG_ROOT);
    lowlevel = lowlevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_LOWLEVEL);

    int iterate = 0;
    for (child = lowlevel->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement())
        iterate++;

    element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);
    lowlevel->InsertEndChild(element);
    lowlevel = lowlevel->LastChildElement();

    lowlevel->InsertEndChild(doc.NewElement(CNS_TAG_OPEN));
    child = lowlevel->LastChildElement();

    Node min;
    min.F = -1;
    int exc = 0;
    for (int i = 0; i < open.size(); ++i) {
        if (open[i].size() > 0) {
            if (open[i].begin()->F <= min.F || min.F == -1) {
                if (open[i].begin()->F == min.F && open[i].begin()->g > min.g) {
                    min = *open[i].begin();
                    exc = i;
                } else if (open[i].begin()->F < min.F || min.F == -1) {
                    min = *open[i].begin();
                    exc = i;
                }
            }
        }
    }
    if (min.F != -1) {
        element = doc.NewElement(CNS_TAG_POINT);
        element->SetAttribute(CNS_TAG_ATTR_X, min.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, min.i);
        element->SetAttribute(CNS_TAG_ATTR_F, min.F);
        element->SetAttribute(CNS_TAG_ATTR_G, min.g);
        if (min.g > 0) {
            element->SetAttribute(CNS_TAG_ATTR_PARX, min.parent->j);
            element->SetAttribute(CNS_TAG_ATTR_PARY, min.parent->i);
        }
        child->InsertEndChild(element);
    }
    for (int i = 0; i < open.size(); ++i) {
        if (open[i].size() > 0) {
            for (auto it = open[i].begin(); it != open[i].end(); ++it) {
                if (it != open[exc].begin()) {
                    element = doc.NewElement(CNS_TAG_POINT);
                    element->SetAttribute(CNS_TAG_ATTR_X, it->j);
                    element->SetAttribute(CNS_TAG_ATTR_Y, it->i);
                    element->SetAttribute(CNS_TAG_ATTR_F, it->F);
                    element->SetAttribute(CNS_TAG_ATTR_G, it->g);
                    if (it->g > 0) {
                        element->SetAttribute(CNS_TAG_ATTR_PARX, it->parent->j);
                        element->SetAttribute(CNS_TAG_ATTR_PARY, it->parent->i);
                    }
                    child->InsertEndChild(element);
                }
                // child->InsertEndChild(element);
            }
        }
    }

    lowlevel->InsertEndChild(doc.NewElement(CNS_TAG_CLOSE));
    child = lowlevel->LastChildElement();

    for (auto it = close.begin(); it != close.end(); ++it) {
        element = doc.NewElement(CNS_TAG_POINT);
        element->SetAttribute(CNS_TAG_ATTR_X, it->second.j);
        element->SetAttribute(CNS_TAG_ATTR_Y, it->second.i);
        element->SetAttribute(CNS_TAG_ATTR_F, it->second.F);
        element->SetAttribute(CNS_TAG_ATTR_G, it->second.g);
        if (it->second.g > 0) {
            element->SetAttribute(CNS_TAG_ATTR_PARX, it->second.parent->j);
            element->SetAttribute(CNS_TAG_ATTR_PARY, it->second.parent->i);
        }
        child->InsertEndChild(element);
    }
}; */

void JsonLogger::writeToLogPath(const std::vector<Node>& path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || path.empty())
        return;
    int iterate = 0;
    nlohmann::ordered_json lplevel;
    for (std::vector<Node>::const_iterator it = path.begin(); it != path.end(); it++) {
        lplevel[iterate] = std::string(CNS_TAG_ATTR_X) + " = " + std::to_string(it->j) + ", " +
                           std::string(CNS_TAG_ATTR_Y) + " = " + std::to_string(it->i) + ", " +
                           std::string(CNS_TAG_ATTR_TIME) + " = " + std::to_string(it->g);
        iterate++;
    }
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_LPLEVEL] = lplevel;
}

void JsonLogger::writeToLogHPpath(const std::vector<Node>& hppath)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || hppath.empty())
        return;
    int partnumber = 0;
    nlohmann::ordered_json hplevel;
    std::vector<Node>::const_iterator iter = hppath.begin();
    std::vector<Node>::const_iterator it = hppath.begin();

    while (iter != --hppath.end()) {
        ++iter;
        hplevel[partnumber] = {{CNS_TAG_ATTR_NUM, partnumber}, {CNS_TAG_ATTR_STX, it->j}, {CNS_TAG_ATTR_STY, it->i},
                               {CNS_TAG_ATTR_FINX, iter->j}, {CNS_TAG_ATTR_FINY, iter->i}, {CNS_TAG_ATTR_LENGTH, iter->g - it->g}};
        ++it;
        ++partnumber;
    }
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_HPLEVEL] = hplevel;
}

void JsonLogger::writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    nlohmann::ordered_json summary;
    summary = {{CNS_TAG_ATTR_NUMOFSTEPS, numberofsteps}, {CNS_TAG_ATTR_NODESCREATED, nodescreated}, {CNS_TAG_ATTR_LENGTH, length},
               {CNS_TAG_ATTR_LENGTH_SCALED, length*cellSize}, {CNS_TAG_ATTR_TIME, std::to_string(time).c_str()}};
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_SUM] = summary;
}

void JsonLogger::writeToLogNotFound()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_PATH] = "Path NOT found!";
}

void JsonLogger::writeToLogObstaclesPath(const std::vector<std::vector<Node>>* obstacles_paths) {
    nlohmann::ordered_json obsPath;
    for (size_t i = 0; i < obstacles_paths->size(); ++i) {
        std::string str = "obstacle id: " + std::to_string(i);
        nlohmann::ordered_json cur;
        int iterate = 0;
        for (size_t j = 0; j < (*obstacles_paths)[i].size(); ++j) {
            cur[iterate] = std::string(CNS_TAG_ATTR_X) + " = " + std::to_string((*obstacles_paths)[i][j].j) + ", " +
                           std::string(CNS_TAG_ATTR_Y) + " = " + std::to_string((*obstacles_paths)[i][j].i) + ", " +
                           std::string(CNS_TAG_ATTR_TIME) + " = " + std::to_string((*obstacles_paths)[i][j].g);
            iterate++;
        }
        obsPath[str] = cur;
    }
    objJson[CNS_TAG_ROOT][CNS_TAG_LOG][CNS_TAG_OBSPATH] = obsPath;
}

// void JsonLogger::setLogFilename(const std::string& file) {
//     LogFileName = file;
// };
