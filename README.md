# Tunnel Optimization

## Description
This project is a final assignment for the course in Combinatorial Optimization. It focuses on designing an efficient system of tunnels that connect various underground resource deposits in 3D space while avoiding underground water reservoirs. The objective is to minimize the total length of the tunnels, ensuring an optimal route that visits all the deposits and returns to the starting point without retracing any path. This problem is a modification of the classic Traveling Salesman Problem (TSP), with additional constraints to avoid specific areas representing underground water reservoirs.

## Problem Description
A mining planning specialist has been tasked with designing an efficient system of tunnels that will connect different underground resource deposits. The challenge is to optimally plan the routes to minimize the tunnel lengths while avoiding areas of underground water reservoirs, which could lead to technical difficulties, ecological risks, or hindered mining operations. The deposits must be connected in such a way that, by traversing the tunnels, it is possible to visit all deposits and return to the starting point without retracing.

The deposits are represented as points $P_i (x_i, y_i, z_i)$, and the water reservoirs are represented as spheres with center $S_i (x_i, y_i, z_i)$ and radius $r_i$.

## Mathematical Description

### Objective Function
The goal is to minimize the total length of the tunnels:

$\text{min}\sum_{i=1}^{n}\sum_{j\neq i, j=1}^{n} d_{ij} e_{ij}$

where:

- $d_{ij}$ is the distance between deposit $i$ and deposit $j$.
- $e_{ij}$ indicates whether a tunnel can be constructed between deposit $i$ and deposit $j$ (0 - cannot be constructed, 1 - can be constructed).

### Distance Calculation
The distance $d_{ij}$ between deposits $i$ and $j$ is given by:

$d_{ij} = \sqrt{(x_i - x_j)^2 + (y_i - y_j)^2 + (z_i - z_j)^2}$

### Collision Condition Check
The equation of the line segment between points $i$ and $j$ can be written as:

$P(t) = P_i + t \cdot (P_j - P_i)$

where $t$ is a parameter, and $P_i$ and $P_j$ are the endpoints of the segment.

The collision condition with a sphere centered at $S_k$ with radius $r_k$ can be expressed using the sphere equation:

$||P(t) - S_k||^2 = r_k^2$

where $||...||$ denotes the Euclidean norm.

By substituting the line segment equation into the sphere equation, we obtain:

$||P_i + t \cdot (P_j - P_i) - S_k||^2 = r_k^2$

This results in a quadratic equation in terms of the parameter $t$:

$a \cdot t^2 + b \cdot t + c = 0$

where:

- $a = ||P_j - P_i||^2$
- $b = 2 \cdot (P_j - P_i) \cdot (P_i - S_k)$
- $c = ||P_i - S_k||^2 - r_k^2$

### Discriminant Calculation
We calculate the discriminant $\Delta$:

$\Delta = b^2 - 4ac$

- If $\Delta < 0$, there is no collision.
- Otherwise, we compute $t_1$ and $t_2$:

$t_1 = \frac{-b + \sqrt{\Delta}}{2a}$

$t_2 = \frac{-b - \sqrt{\Delta}}{2a}$

- If $(0 \leq t_1 \leq 1)$ or $(0 \leq t_2 \leq 1)$, then the segment between points $P_i$ and $P_j$ collides with the sphere centered at $S_k$ with radius $r_k$.

### Tunnel Construction Condition
The condition to determine whether a tunnel can be constructed between points $i$ and $j$ is as follows:

$e_{ij} = 0 \quad \text{if} \quad ∃_{k}∃_{t_{1}}∃_{t_{2}} \quad (0 \leq t_1 \leq 1) \lor (0 \leq t_2 \leq 1), \quad \text{otherwise} \quad e_{ij} = 1$

## Acknowledgment

- [Travelling salesman problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem)
- [Brute-force search](https://en.wikipedia.org/wiki/Brute-force_search)
- [Greedy algorithm](https://en.wikipedia.org/wiki/Greedy_algorithm)
- [Metaheuristic](https://en.wikipedia.org/wiki/Metaheuristic)
- [Tabu search](https://en.wikipedia.org/wiki/Tabu_search)
- [Matplotlib](https://matplotlib.org/)
- [NumPy](https://numpy.org/)
- [nlohmann/json](https://github.com/nlohmann/json)

## License

This project is open-source under the MIT License.
