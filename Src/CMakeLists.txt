set(HEADERS
    Agent.h
    Config.h
    Constants.h
    Heap.h
    JsonLogger.h
    Map.h
    Mission.h
    Node.h
    Parse.h
    Pbs.h
    Searchresult.h
    Sipp-ip.h
    Tasks.h
    PTNode.h
    Primitive.h
    Move.h 
)

set(SOURCES
    Agent.cpp
    Config.cpp
    Heap.cpp
    Map.cpp
    Mission.cpp
    Node.cpp
    Parse.cpp
    Pbs.cpp
    Sipp-ip.cpp
    JsonLogger.cpp
    json.hpp
    Tasks.cpp
)

add_library(PPLIB STATIC ${SOURCES} ${HEADERS})
target_include_directories(PPLIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(Search main.cpp)
target_link_libraries(Search PRIVATE PPLIB)

option(ENABLE_ASAN "Enable AddressSanitizer for debugging memory errors" OFF)

if(ENABLE_ASAN)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(STATUS "AddressSanitizer (ASan) has been enabled for targets: PPLIB, Search")

        target_compile_options(PPLIB PRIVATE -fsanitize=address -g -fno-omit-frame-pointer)
        target_link_libraries(PPLIB PRIVATE -fsanitize=address)

        target_compile_options(Search PRIVATE -fsanitize=address -g -fno-omit-frame-pointer)
        target_link_libraries(Search PRIVATE -fsanitize=address)

    else()
        message(WARNING "ENABLE_ASAN is ON, but the current compiler is not GCC or Clang. ASan will not be enabled.")
    endif()
endif()


