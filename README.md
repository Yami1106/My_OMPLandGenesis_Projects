<div align="center">

# TAMP Framework — PDDL → OMPL → Franka Panda

[![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)](https://isocpp.org)

*A complete Task and Motion Planning stack: symbolic PDDL planning feeds into OMPL motion planning, executed on a Franka Emika Panda in the Genesis simulator — with closed-loop re-planning on failure.*

</div>

---

## System architecture

```
PDDL domain + problem
        ↓
   Pyperplan  (symbolic planner)
        ↓
 Sequence of actions  →  OMPL motion planner
        ↓
  Genesis simulator  →  Franka Panda (7-DOF)
        ↓
 Predicate extraction → Re-grounding if failed
```

---

## Predicates

`ON` · `ONTABLE` · `CLEAR` · `HOLDING` · `HANDEMPTY` · `ADJACENT-X` · `ADJACENT-Y`

---

## Goals achieved

| Goal | Task | Success rate |
|---|---|---|
| 1 | Two 3-block towers | 100% |
| 2 | 5-block tower | 100% |
| 3 | Tallest possible tower (re-planning on topple) | 7 blocks max |
| 4.1 | Pentagon tower (2-layer bridged structure) | Stability challenge |
| 4.2 | Directional adjacency grid (2×2 base + 2 stacked) | ADJACENT-X/Y predicates |

---

## Key technical features

- **Closed-loop re-grounding** — detects failure, extracts current world state, re-runs Pyperplan
- **Attached object handling** — OMPL planner accounts for objects held by the gripper
- **Collision-free motion primitives** for pick-and-place with full arm planning
- **Predicate extraction** from simulator state to keep symbolic and geometric layers in sync

---

## Tech stack

`Python` · `C++` · `OMPL` · `Pyperplan` · `Genesis Simulator` · `PDDL`

---

<div align="center">
WPI Motion Planning (RBE 550) · <a href="https://github.com/Yami1106">Ashish Sukumar</a>
</div>
<!-- -->
