# Empirical Evaluation of Primitives in MAPF with Kinodynamic Constraints

This repository presents an empirical evaluation of various families of motion primitives for the Multi-Agent Path Finding (MAPF) problem with kinodynamic constraints.  
The experiments are based on two algorithms:

- [PSB](https://github.com/JingtianYan/PSB-RAL) (Priority-Based Search with Safe Interval Path Planning and Bezier Curves Optimization)
- [PBS](https://arxiv.org/pdf/1812.06356) + [SIPP-IP](https://github.com/PathPlanning/SIPP-IP) (Priority-Based Search and Safe Interval Path Planning with Intervals Projection)

A detailed description of the methods, primitive families, and comparison results is available in our paper [TBD].

---

## 🗂️ Repository Structure

- `Demos/` — animations showcasing different families of motion primitives in use.
- `Examples/` — input/output JSON examples for running algorithms on custom tasks (not from MovingAI).
- `Instances/` — datasets with maps and scenarios from the [MovingAI benchmark](https://movingai.com/benchmarks/mapf/index.html) used in our experiments.
- `Src/` — C++ source files for PBS + SIPP-IP and primitives implementation.

---

## 📦 Main Components

| Class       | Responsibility |
|-------------|----------------|
| Agent     | Holds per-agent data |
| Config    | Configures algorithm parameters (e.g., planning horizon, metric) |
| `JsonLogger`| Logs algorithm output to a JSON file (using [nlohmann/json](https://github.com/nlohmann/json)) |
| Map       | Stores map data |
| Mission   | Manages end-to-end execution of the algorithm |
| Parse     | Converts user inputs into a JSON format for algorithm execution |
| PBS       | Implements the Priority-Based Search algorithm and defines the motion primitive set |
| SIPP-IP   | Modified Safe Interval Path Planning with Interval Projection (based on [authors' implementaion](https://github.com/PathPlanning/SIPP-IP)) |
| Task      | Stores the current objective for each agent |

---

## 🚀 Getting Started

### Requirements

- C++20 compatible compiler (e.g., GCC 10+, Clang 11+)
- CMake 3.16+  
  You can install CMake via package manager or from cmake.org

### Building the Project

```bash git clone https://github.com/Pr0tuberanec/Empirical-Evaluation-of-Primitives-in-MAPF-with-Kinodynamic-Constraints-.git
cd Empirical-Evaluation-of-Primitives-in-MAPF-with-Kinodynamic-Constraints-
mkdir build && cd build
cmake ..
make
```

### Launch
You can launch the project in two ways:

#### Option 1: JSON-based input (custom task)
Use your own .json file describing the map and agents, example you can search at Examples. Run the planner like this:

```bash
./Search path/to/task.json
```

By default, the output will be saved to task_log.json in the same folder.

#### Option 2: Using MovingAI input format
You can provide a .map file and .scen file (e.g., from the Instances/ folder). You may also pass:
an optional config.json for algorithm parameters, an optional number of agents to include from the .scen file (50 by default)

Run the planner like this:

```bash
./Search path/to/map.map path/to/task.scen [path/to/config.json] [number_of_agents]
```
The output will be saved as <task_file>_log.json in the same folder.
