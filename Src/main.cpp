#include "Mission.h"

int main(int argc, char* argv[])
{
    int num_agents = 50; // default number of agents
    fs::path task = prepareTaskFromArgs(argc, argv, num_agents);
    if (task.empty()) return 1;

    std::string task_str = task.string();
    Mission mission(task_str.c_str());

    std::cout<<"Parsing the map from JSON:"<<std::endl;

    if(!mission.getMap()) {
        std::cerr<<"Incorrect map! Program halted!"<<std::endl;
    }
    else {
        std::cout<<"Map OK!"<<std::endl<<"Parsing configurations (algorithm, log) from JSON:"<<std::endl;
    
        if(!mission.getConfig())
            std::cerr<<"Incorrect configurations! Program halted!"<<std::endl;
        else {
            std::cout<<"Configurations OK!"<<std::endl<<"Parsing tasks:"<<std::endl;
        
            if(!mission.getTasks())
                std::cerr<<"Incorrect tasks! Program halted!"<<std::endl;
            else {
                std::cout<<"Tasks OK!"<<std::endl<<"Creating log channel:"<<std::endl;
        
                if(!mission.createLog())
                    std::cerr<<"Log chanel has not been created! Program halted!"<<std::endl;
                else {
                    std::cout<<"Log OK!"<<std::endl<<"Start searching the path:"<<std::endl;

                    mission.startSearch();

                    std::cout<<"Search is finished!"<<std::endl;

                    mission.printSearchResultsToConsole();
                    mission.saveSearchResultsToLog();

                    std::cout<<"Results are saved (if chosen) via created log channel."<<std::endl;
                }
            }
        }
    }
    return 0;
}

