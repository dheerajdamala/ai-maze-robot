# Smart Maze-Solving Robot | AI with Python

An intelligent 2D kinematic maze-solving robot demonstrating search heuristics and graph exploration algorithms studied in the **AI with Python** course.

**Live Web Application**: [https://ai-maze-robot.netlify.app](https://ai-maze-robot.netlify.app)

---

## 1. Problem Formulation

* **Initial State**: Robot located at start cell $S(r_0, c_0) = (1, 1)$ with zero initial velocity and orientation heading $\theta = 0^\circ$.
* **Goal State**: Target destination cell $G(r_g, c_g) = (29, 29)$ in an $N \times M$ grid ($31 \times 31$).
* **State Space**: A discrete 2D spatial coordinate grid $S = \{ (r, c) \mid 0 \le r < 31, 0 \le c < 31 \}$.
* **Action Space**: Orthogonal kinematic transitions $A = \{ \text{Up } (-1, 0), \text{Down } (+1, 0), \text{Left } (0, -1), \text{Right } (0, +1) \}$.
* **Constraints**: 
  - Wall collision avoidance: $\text{Grid}[r][c] \neq \text{WALL}$.
  - Grid boundary containment: $0 \le r < 31$ and $0 \le c < 31$.
* **Heuristic Functions**:
  - **Manhattan Distance**: $h(n) = |r_n - r_g| + |c_n - c_g|$ (Admissible & Consistent).
  - **Euclidean Distance**: $h(n) = \sqrt{(r_n - r_g)^2 + (c_n - c_g)^2}$ (Admissible).

---

## 2. Implemented AI Algorithms

1. **A* Search (Manhattan)**: Evaluates $f(n) = g(n) + h(n)$ using $L_1$ norm. Guarantees optimal shortest path while minimizing node expansions.
2. **A* Search (Euclidean)**: Evaluates $f(n) = g(n) + h(n)$ using straight-line distance.
3. **Breadth-First Search (BFS)**: Unweighted queue-based frontier exploration; guarantees minimal step count.
4. **Depth-First Search (DFS)**: LIFO stack-based deep explorer; demonstrates non-optimal memory-efficient exploration.
5. **Uniform Cost Search (UCS / Dijkstra)**: Priority queue ordering by cumulative path cost $g(n)$.
6. **Greedy Best-First Search**: Explores strictly by minimal heuristic $h(n)$; rapid execution with potential sub-optimality.

---

## 3. Environment Generation

* **Recursive Backtracker**: Generates deep labyrinthine mazes with single solution corridors.
* **Prim's Algorithm**: Generates organic branching mazes with high junction density.
* **Random Obstacles**: Configurable density fields ($25\%$ and $35\%$) for obstacle avoidance.
* **Presets**: Bottleneck Choke, Spiral Labyrinth, Multi-Room Chambers, and Local Minima Traps.
* **Interactive Tooling**: Real-time obstacle drawing, wall eraser, and dynamic Start/Goal placement.

---

## 4. Performance Benchmarks

The built-in multi-algorithm benchmark tool executes all 6 algorithms on identical grid configurations, plotting:
* **Explored States** (Search efficiency)
* **Path Length** (Solution optimality)
* **Execution Time** (Milliseconds)
* **Peak Frontier Size** (Memory footprint)

---

## 5. Project Structure

```
ai-maze-robot/
├── index.html            # Standalone web visualizer
├── css/
│   └── style.css         # UI stylesheet
├── js/
│   ├── maze_robot.js     # 6 AI algorithms & kinematic 2D robot engine
│   ├── comparison.js     # Chart.js multi-algorithm benchmark module
│   └── app.js            # UI controller
├── ai with python/       # Desktop Python application (Tkinter)
│   ├── gui.py            # Desktop visualizer
│   ├── maze_generator.py # Procedural maze algorithms
│   ├── robot.py          # Robot kinematics
│   └── algorithms/       # Search algorithm implementations
└── netlify.toml          # Web deployment configuration
```

---

## 6. How to Run

### Web Application
Open `index.html` in any web browser, or visit [https://ai-maze-robot.netlify.app](https://ai-maze-robot.netlify.app).

### Desktop Application (Python)
```bash
cd "ai with python"
pip install -r requirements.txt
python gui.py
```

