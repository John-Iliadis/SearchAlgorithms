# High-distinction project implemented for COS30019.

Given a 2D grid as input, the program will attempt to find a solution with a specified search method.

The following search algorithms have been implemented:
- Breadth first search
- Depth first search
- Iterative deepening search
- Uniform cost search
- A* search
- Bidirectional A* search


## Example command
> py main.py {filename} {search method}

The search method can be one of [bfs, dfs, cus1 (for iterative deepening), ucs, as, cus2 (for bidirectional A*)]

## Example input file

**Problem:**

```python
[6, 11]  # rows, columns
(0, 5)  # coordinates of initial position
(5, 5) | (10, 0)  # coordinates of goal positions
(0, 2, 1, 2)  # this tuple defines a wall. The first two parameters are the coordinates. The last two are the width and height.
(2, 0, 1, 2)
(3, 1, 2, 1)
(2, 3, 1, 3)
(3, 5, 2, 1)
(4, 4, 3, 1)
(6, 2, 3, 1)
(8, 1, 1, 1)
(8, 3, 1, 2)
(9, 4, 1, 1)
```

<br>

**Problem visualization:**

![problem](problem.PNG)
