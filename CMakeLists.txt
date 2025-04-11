cmake_minimum_required(VERSION 3.12)
project(Asearch  # Используем ваше оригинальное имя
        LANGUAGES CXX)

# Настройки компилятора
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Флаги для Debug/Release
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Автоматическое добавление всех исходников
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "Src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "Src/*.h")

# Главный исполняемый файл
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
)

# Пути для заголовков
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Src
)