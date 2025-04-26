
set(HEADERS         agent.h
                    astar.h
                    config.h
                    dynobs.h
                    environmentoptions.h
                    gl_const.h
                    heap.h
                    ilogger.h
                    map.h
                    mission.h
                    node.h
                    pbs.h
                    search.h
                    searchresult.h
                    sipp.h
                    tinyxml2.h
                    xmllogger.h)

set(SOURCES         agent.cpp
                    astar.cpp
                    config.cpp
                    dynobs.cpp
                    environmentoptions.cpp
                    heap.cpp
                    json.hpp
                    main.cpp
                    map.cpp
                    mission.cpp
                    pbs.cpp
                    search.cpp
                    sipp.cpp
                    tinyxml2.cpp
                    xmllogger.cpp )

add_library(PPLIB STATIC ${SOURCES} ${HEADERS})  # Создаём статическую библиотеку
target_include_directories(PPLIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})