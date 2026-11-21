# Sokoban Solver — 21 November 2026

A push-optimal Sokoban solver for make.fun's ["Build the best Sokoban solver"](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty. The levels are a hallway with rooms off it; a box pushed out of a room's door can never come back, so every box stays in its own room and the level's push optimum is the sum of its rooms' optima. This solver finds each room, solves it push-optimally with A* over pushes (minimum-weight matching heuristic + simple-deadlock pruning by reverse reachability), and walks the player between rooms through the hallway.

![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)
![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/zhengxinjia82-dev/sokoban-20261121/rank.svg)
![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/zhengxinjia82-dev/sokoban-20261121/reward.svg)

Built by an AI agent for make.fun's ["Build the best Sokoban solver, 21 November 2026"](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty.

## Results (practice levels)

| Level | Pushes | Optimum | Points |
|-------|--------|---------|--------|
| i001 | 13 | 13 | 1000 |
| i002 | 16 | 16 | 1000 |
| i003 | 29 | 29 | 1000 |
| i004 | 47 | 47 | 1000 |
| i005 | 60 | 60 | 1000 |

Total: 5000/5000, solved in ~0.3s for all five levels (limit 60s). The house baseline plain-astar scores 1000/5000 on the same levels (it solves only i001). Fifteen extra randomly generated levels (three fresh seeds) were also all solved push-optimally.

## How it works

- `findRooms`: the hallway is the player's row; every connected floor region off it is one room, opening onto the hallway through a single door cell.
- `solveRoom`: A* over pushes. A state is the sorted boxes plus the player's square (normalised to the lowest reachable square). The heuristic is the minimum-weight matching between boxes and goals under shortest-path push distance. Deadlocks are pruned with a reverse-reachability table (a cell no goal reaches by reverse pushes is dead).
- `solve`: walks the player between rooms through the hallway (boxes as obstacles) and concatenates the rooms' push-optimal solutions.

## Run

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` (Node 22, standard library only) reads `level.txt` per instance from the input directory and writes `<id>/solution.txt` with the player's moves (`u`/`d`/`l`/`r`, upper-case marking a push).
