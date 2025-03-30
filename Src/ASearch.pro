TARGET = pathplanning
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11 -O2 -Wall -Wextra

win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}

SOURCES += \
    astar.cpp \
    config.cpp \
    environmentoptions.cpp \
    heap.cpp \
    main.cpp \
    map.cpp \
    mission.cpp \
    search.cpp \
    tinyxml2.cpp \
    xmllogger.cpp 
     
HEADERS += \
    astar.h \
    config.h \
    dynobs.h \
    environmentoptions.h \
    gl_const.h \
    heap.h \
    ilogger.h \
    map.h \
    mission.h \
    node.h \
    search.h \
    searchresult.h \
    sipp.h \
    tinyxml2.h \
    xmllogger.h