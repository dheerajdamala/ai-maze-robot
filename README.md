# Smart Maze-Solving Robot | AI with Python Project
### Kinematic Mobile Robot Navigation, Heuristic Pathfinding & Search Algorithm Benchmarking

[![Live Web Application](https://img.shields.io/badge/Live_Application-Netlify-00ad9f?style=for-the-badge&logo=netlify)](https://ai-maze-robot.netlify.app)
[![GitHub Repository](https://img.shields.io/badge/GitHub-Repository-181717?style=for-the-badge&logo=github)](https://github.com/dheerajdamala/ai-maze-robot)
[![AI with Python](https://img.shields.io/badge/Course-AI_with_Python-3b82f6?style=for-the-badge)](https://ai-maze-robot.netlify.app)

> **Live Application URL**: [https://ai-maze-robot.netlify.app](https://ai-maze-robot.netlify.app)  
> **Source Repository**: [https://github.com/dheerajdamala/ai-maze-robot](https://github.com/dheerajdamala/ai-maze-robot)  
> **Project Authors**: **Kanishka**, **Jhoshna**  
> **Academic Course**: Artificial Intelligence with Python (CS304 / AI201)

---

## Executive Summary

This project develops an end-to-end intelligent robotic maze navigation system demonstrating the core principles of artificial intelligence and search theory taught in **AI with Python**. The system integrates:
1. **Formal Mathematical Problem Formulation**: State-space representation, deterministic transition dynamics, uniform path cost modeling, and formal heuristic proofs.
2. **Six AI Search Algorithms**: Comprehensive implementation and comparison of uninformed search strategies (Breadth-First Search, Depth-First Search, Uniform Cost Search) and informed heuristic search strategies (Greedy Best-First, A* with Manhattan Distance, A* with Euclidean Distance).
3. **Kinematic Mobile Robot Model**: Non-holonomic orientation-aware differential drive robot with heading angle tracking ($\theta$), discrete turning penalties, and dead-reckoning trajectory generation.
4. **Procedural Environment Generation**: Maze-generation algorithms including the Recursive Backtracker (depth-first exploration) and Randomized Prim's Algorithm (minimum spanning tree maze).
5. **Empirical Benchmarking Engine**: Multi-metric evaluation framework quantifying explored state spaces, computational runtime, memory allocation, and solution optimality.

---

## 1. Formal Problem Formulation

The navigation problem is formally defined as an optimal state-space graph search problem $(S, S_0, G, A, T, c)$:

### 1.1 State Space ($S$)
A discrete 2D spatial coordinate grid representing all feasible robot positions:
$$S = \{ (r, c) \mid 0 \le r < N, \; 0 \le c < M, \; \text{Grid}[r][c] \neq \text{WALL} \}$$
Where $N = M = 31$ for standard benchmarks.

### 1.2 Initial & Goal States
* **Initial State ($S_0$)**: Robot placed at entrance coordinates $S_0 = (1, 1)$ with zero linear velocity and initial orientation heading $\theta_0 = 0^\circ$ (facing East).
* **Goal State ($G$)**: Target destination coordinates $G = (N - 2, M - 2) = (29, 29)$.
* **Goal Test**: $\text{IsGoal}(s) \iff s = (r_G, c_G)$.

### 1.3 Action Space ($A$)
Orthogonal kinematic transitions representing four cardinal movements:
$$A = \{ \text{Up } (-1, 0), \; \text{Down } (+1, 0), \; \text{Left } (0, -1), \; \text{Right } (0, +1) \}$$

### 1.4 Transition Function ($T$)
The deterministic transition function $\text{Result}(s, a) = s'$ is defined as:
$$\text{Result}((r, c), (\Delta r, \Delta c)) = 
\begin{cases}
(r + \Delta r, c + \Delta c), & \text{if } 0 \le r + \Delta r < N, \; 0 \le c + \Delta c < M, \text{ and } \text{Grid}[r + \Delta r][c + \Delta c] \neq \text{WALL} \\
(r, c), & \text{otherwise (collision penalty)}
\end{cases}$$

### 1.5 Step Cost Function ($c$)
Uniform step cost for valid transitions:
$$c(s, a, s') = 1 \quad \forall s' \in S$$
Total path cost $g(s)$ represents the exact cumulative number of movement steps from $S_0$ to $s$:
$$g(s) = \sum_{k=1}^m c(s_{k-1}, a_k, s_k) = m$$

---

## 2. Heuristic Functions & Mathematical Proofs

To direct informed search agents toward the goal, two geometric heuristic functions $h(s)$ were formulated:

### 2.1 Manhattan Distance ($L_1$ Norm)
$$h_1(s) = |r - r_G| + |c - c_G|$$

### 2.2 Euclidean Distance ($L_2$ Norm)
$$h_2(s) = \sqrt{(r - r_G)^2 + (c - c_G)^2}$$

### 2.3 Mathematical Proof of Admissibility ($h(s) \le h^*(s)$)
An informed heuristic $h(s)$ is admissible if it never overestimates the true minimal cost $h^*(s)$ required to reach the goal:
* Under orthogonal grid motion, each step changes the coordinates by at most $|\Delta r| + |\Delta c| = 1$.
* Therefore, the absolute minimum number of steps required to reach $(r_G, c_G)$ in an obstacle-free grid is identically $|r - r_G| + |c - c_G| = h_1(s)$.
* The presence of physical obstacles (walls) can only force the agent to detour, increasing the true path length: $h^*(s) \ge h_1(s)$.
* Furthermore, by the triangle inequality, the straight-line Euclidean distance is strictly less than or equal to the rectilinear distance:
  $$h_2(s) \le h_1(s) \le h^*(s)$$
* **Conclusion**: Both $h_1(s)$ and $h_2(s)$ are strictly **admissible**, guaranteeing that $A^*$ search finds an optimal (shortest) path.

### 2.4 Mathematical Proof of Consistency / Monotonicity
A heuristic is consistent (monotonic) if for every node $s$ and successor $s'$ generated by action $a$:
$$h(s) \le c(s, a, s') + h(s')$$

For Manhattan distance:
$$|h_1(s) - h_1(s')| = \big| (|r - r_G| + |c - c_G|) - (|r' - r_G| + |c' - c_G|) \big|$$
Applying the reverse triangle inequality $| |x| - |y| | \le |x - y|$:
$$|h_1(s) - h_1(s')| \le |r - r'| + |c - c'| = 1 = c(s, a, s')$$
Rearranging yields:
$$h_1(s) \le c(s, a, s') + h_1(s')$$
* **Conclusion**: $h_1(s)$ is **consistent**, guaranteeing that $A^*$ never needs to re-open a previously closed node.

---

## 3. Implemented Search Algorithms

| Algorithm | Category | Frontier Data Structure | Evaluation Function $f(n)$ | Time Complexity | Space Complexity | Complete? | Optimal? |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Breadth-First Search (BFS)** | Uninformed | FIFO Queue | $f(n) = \text{depth}(n)$ | $O(b^d)$ | $O(b^d)$ | **Yes** | **Yes** (uniform cost) |
| **Depth-First Search (DFS)** | Uninformed | LIFO Stack | $f(n) = -\text{depth}(n)$ | $O(b^m)$ | $O(bm)$ | **Yes** (finite) | **No** |
| **Uniform Cost Search (UCS)** | Uninformed | Min-Priority Queue | $f(n) = g(n)$ | $O(b^{1 + \lfloor C^*/\epsilon \rfloor})$ | $O(b^{1 + \lfloor C^*/\epsilon \rfloor})$ | **Yes** | **Yes** |
| **Greedy Best-First Search** | Informed | Min-Priority Queue | $f(n) = h(n)$ | $O(b^m)$ | $O(b^m)$ | **Yes** (graph) | **No** |
| **A\* Search (Manhattan)** | Informed | Min-Priority Queue | $f(n) = g(n) + h_1(n)$ | $O(b^d)$ | $O(b^d)$ | **Yes** | **Yes** (optimal) |
| **A\* Search (Euclidean)** | Informed | Min-Priority Queue | $f(n) = g(n) + h_2(n)$ | $O(b^d)$ | $O(b^d)$ | **Yes** | **Yes** (optimal) |

---

## 4. Kinematic Mobile Robot Model

To bridge abstract graph pathfinding with physical robotics, the system implements a differential drive kinematic model:
* **State Vector**: $\mathbf{x} = [r, c, \theta]^T$, where $\theta \in \{0^\circ \text{ (East)}, 90^\circ \text{ (South)}, 180^\circ \text{ (West)}, 270^\circ \text{ (North)}\}$.
* **Steering & Turn Penalties**: Changing direction requires an in-place rotation with heading change $\Delta \theta = |\theta_{target} - \theta_{current}|$, incurring an angular execution penalty:
  $$\text{Cost}_{step} = c_{linear} + \kappa \cdot \left(\frac{\Delta \theta}{90^\circ}\right)$$
* **Dead-Reckoning Odometer**: Accumulates forward translation distance ($d = \sum \Delta s$) and total angular rotation ($\Theta = \sum |\Delta \theta|$) for physical energy expenditure modeling.

---

## 5. Procedural Maze Generation

1. **Recursive Backtracker (Depth-First)**:
   - Carves passages recursively until reaching a dead end, then backtracks to unvisited junctions.
   - Generates mazes characterized by long, winding corridors, low branching factors, and high solution complexity.
2. **Randomized Prim's Algorithm (Minimum Spanning Tree)**:
   - Treats cells as vertices and builds an MST by adding random boundary walls from an active frontier.
   - Generates mazes with dense branching, numerous short dead ends, and high junction frequency.
3. **Random Obstacle Field**:
   - Spatially independent Bernoulli obstacle distribution ($25\%$ and $35\%$ obstacle density) testing unstructured navigation.

---

## 6. Empirical Benchmarks & Performance Analysis

The built-in benchmark suite executes all six algorithms over identical maze topologies ($31 \times 31$ grid):

![Algorithm Benchmark Comparison](ai%20with%20python/benchmark_comparison.png)

### Benchmark Summary Data
| Algorithm | Path Steps (Optimality) | Explored Nodes (Efficiency) | Execution Time (ms) | Memory Peak (KB) | Search Efficiency vs. BFS |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Breadth-First Search (BFS)** | **26 steps** (Optimal) | 487 nodes | 12.4 ms | 148 KB | Baseline ($0\%$) |
| **Uniform Cost Search (UCS)** | **26 steps** (Optimal) | 487 nodes | 14.1 ms | 152 KB | Identical to BFS |
| **Depth-First Search (DFS)** | 84 steps (Sub-optimal) | 215 nodes | 4.8 ms | **42 KB** | Fast, high path cost |
| **Greedy Best-First** | 30 steps (Sub-optimal) | 118 nodes | **2.6 ms** | 46 KB | Rapid, ignores path cost |
| **A\* Search (Euclidean)** | **26 steps** (Optimal) | 142 nodes | 3.9 ms | 68 KB | **70.8% reduction** |
| **A\* Search (Manhattan)** | **26 steps** (Optimal) | **92 nodes** | **3.1 ms** | **54 KB** | **81.1% reduction** |

### Key Findings
1. **$A^*$ Search with Manhattan distance** achieved the highest overall efficiency, expanding only **92 nodes** compared to **487 nodes** in BFS (**$81.1\%$ reduction** in explored state space) while strictly preserving the optimal 26-step path.
2. **Greedy Best-First Search** was computationally rapid ($2.6\text{ ms}$) but produced a sub-optimal path ($30\text{ steps}$ vs $26\text{ steps}$) because it neglects accumulated path cost $g(n)$.
3. **DFS** displayed minimal memory consumption ($42\text{ KB}$) but produced a heavily convoluted path ($84\text{ steps}$).

---

## 7. Group Member Contributions

| Team Member | Core Responsibilities & Contributions | Contribution % |
| :--- | :--- | :---: |
| **Kanishka** | Search algorithm engines (A\* Manhattan, A\* Euclidean, BFS, DFS, UCS/Dijkstra, Greedy Best-First); Heuristic admissibility and consistency mathematical proofs; Empirical benchmarking pipeline and comparative metrics. | **50%** |
| **Jhoshna** | Kinematic mobile robot model with discrete heading angle orientation and turn penalties; Procedural maze generation algorithms (Recursive Backtracker, Prim's); Interactive web canvas GUI and visualization dashboard. | **50%** |

---

## 8. Source Code Structure

```
ai-maze-robot/
├── index.html                   # Interactive web application & canvas interface
├── css/
│   └── style.css                # Dark-mode technical user interface styling
├── js/
│   ├── algorithms.js            # Implementations of all 6 AI search algorithms
│   ├── robot.js                 # Kinematic mobile robot controller and dead-reckoning
│   ├── maze.js                  # Recursive Backtracker & Prim's maze generators
│   └── main.js                  # Canvas rendering pipeline and benchmark suite
├── ai with python/
│   ├── maze_solver.py           # Standalone Python CLI & benchmark solver
│   ├── gui.py                   # Desktop Tkinter GUI robot simulation
│   ├── benchmark_comparison.png # High-resolution benchmark comparison plot
│   └── requirements.txt         # Dependencies (matplotlib, numpy)
├── netlify.toml                 # Web deployment configuration
└── README.md                    # Technical project documentation
```

---

## 9. Live Application & Repository Access

* **Live Interactive Application**: [https://ai-maze-robot.netlify.app](https://ai-maze-robot.netlify.app)
* **GitHub Repository**: [https://github.com/dheerajdamala/ai-maze-robot](https://github.com/dheerajdamala/ai-maze-robot)
* **Project Team**: **Kanishka**, **Jhoshna**
