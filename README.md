# Motion Planning for Car & Robotic Arm  
A collection of ROS-based motion-planning demos implementing **A\***, **RRT**, for a wheeled vehicle 

## Table of Contents

- [Overview](#overview)  
- [Features](#features)  
- [Demos](#demos) 
- [Acknowledgments](#acknowledgments)

---

## Overview

This repository contains algorithms for grid based motion planning using A* and RRT. The problems are based in R^2 and utilizes a a [NetworkX](https://networkx.org/) graph for the the mapping

## Features

- **RRT Planner**: Basic sampling-based planner with `--show-tree` visualization  
- **A\***: Optimal grid search with customizable sampler (`lattice` vs. `Halton`)
- **tradeoff-analysis**: 

---

## Demos

### Car Planning (2D)

![rrt_path](https://github.com/user-attachments/assets/e23f9192-3e73-4f54-87a5-e6b4ae235ba7)
*RRT path on `map1.txt

![astar_path](https://github.com/user-attachments/assets/3608ef90-6096-4f75-bb70-6fb460fce563)
*A\* path on the same map using a lattice sampler.

---

### core implemention 
take a look at the core implementation
[**Plannning**]planning/src/planning/problems.py

Initially developed for Cornell’s CS4750 Robotic Foundations course.[^0]
[^0]: https://www.cs.cornell.edu/courses/cs5750/2024fa/
``` :contentReference[oaicite:3]{index=0}  
