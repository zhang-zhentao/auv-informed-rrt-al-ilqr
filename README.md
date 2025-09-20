# Autonomous Motion Planning and Control for an Underwater Vehicle (Informed RRT + Constrained iLQR)

This repository contains the **research code** accompanying the paper:

> **Autonomous motion planning and control for an underwater vehicle based on informed RRT and constrained Iterative LQR**

It provides:
- **Informed RRT\*** for shortest-path planning in 3D clutter,
- **KD-tree–accelerated neighbor search** with **lazy rebuild** and **tail compensation**,
- A **unified collision predicate** covering spheres, AABBs, and convex polyhedra,
- **Equal-arc resampling** to bridge planning and control, and
- **Augmented-Lagrangian iLQR (AL-iLQR)** tracking with a simple **PID warm start**.
---

## Features

- **Objective**: Euclidean arclength.
- **Neighbors**: 3D RRT\* schedule \( r_n = \gamma (\log n / n)^{1/3} \).
- **Indexing**: KD-tree + **lazy rebuild** + **tail compensation** (exact results with amortized sublinear queries).
- **Collision**:
  - Sphere–segment distance check,
  - AABB face intersection tests,
  - Convex hull triangle tests with barycentric inclusion.
- **Interface**: Equal-arc resampling for stable tracking.
- **Control**: AL-iLQR with box constraints on \((u, r, q)\); PID warm start.

---

## Repository Layout (core only)
