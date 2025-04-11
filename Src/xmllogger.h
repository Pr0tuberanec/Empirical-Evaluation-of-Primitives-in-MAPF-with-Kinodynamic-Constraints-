#ifndef XMLLOGGER_H
#define XMLLOGGER_H

#include "json.hpp"
#include "ilogger.h"
#include "node.h"
#include <vector>

    // Класс для вывода данных в JSON
    class JsonLogger : public ILogger {
    public:
        explicit JsonLogger(const std::string& loglevel) : ILogger(loglevel) {}

        ~JsonLogger() override = default;

        bool getLog(const char* fileName, const std::string* logParams) override;
        void saveLog() override;

        //void writeToLogOpenClose(const std::vector<std::list<Node> > &open,
        //                         const std::unordered_map<int, Node> &close,
        //                         bool last);
        void writeToLogMap(const Map& map, const std::vector<Node>& path) override;
        void writeToLogObstaclesPath(const std::vector<std::vector<Node>>* obstacles_paths) override;
        void writeToLogPath(const std::vector<Node>& path) override;
        void writeToLogHPpath(const std::vector<Node>& hppath) override;
        void writeToLogNotFound() override;

        void writeToLogSummary(unsigned int numberOfSteps, unsigned int nodesCreated,
                               float length, double time, double cellSize) override;

        // void setLogFilename(const std::string& file) override;

    private:
        std::string LogFileName;
        nlohmann::ordered_json objJson;
    };

#endif // XMLLOGGER_H
