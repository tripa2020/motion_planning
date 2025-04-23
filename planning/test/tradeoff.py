#!/usr/bin/env python

import networkx as nx
import numpy as np
import os
import rosunit
import time
import threading
import unittest

from planning import search
from planning.samplers import LatticeSampler
from planning.problems import R2Problem
from planning.roadmap import Roadmap

class TestTradeoff(unittest.TestCase):

    def test_astar_beats_rrt(self):
        
        np.random.seed(111)
        
        fname = os.path.join(os.path.dirname(__file__), "share", "astar_beats_rrt_map.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        problem = R2Problem(permissible_region)
        sampler = LatticeSampler(problem.extents)

        num_vertices = 100
        connection_radius = 3.0
        rm = Roadmap(problem, sampler, num_vertices, connection_radius)
        
        start = np.array([1, 1])
        goal = np.array([19, 19])
        start_id = rm.add_node(start, is_start=True)
        goal_id = rm.add_node(goal, is_start=False)

        try:
            astar = search.ASTARPlanner(rm)
            _ = astar.Plan(start_id, goal_id)
        except nx.NetworkXNoPath as e:
            self.fail("A* failed to find a path")
        
        batch_size = 100
        rrt = search.RRTPlanner(problem, permissible_region, batch_size=batch_size)

        # Create thread for RRT
        rrt_thread = threading.Thread(target=rrt.Plan, args=(start, goal), daemon=True)
        rrt_thread.start()
        rrt_thread.join(10)
        assert rrt_thread.is_alive(), "RRT found a path in less than 10 seconds"
    
    def test_rrt_beats_astar(self):
        
        np.random.seed(111)
        
        fname = os.path.join(os.path.dirname(__file__), "share", "rrt_beats_astar_map.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        problem = R2Problem(permissible_region)
        sampler = LatticeSampler(problem.extents)

        num_vertices = 100
        connection_radius = 3.0
        rm = Roadmap(problem, sampler, num_vertices, connection_radius)
        
        start = np.array([1, 1])
        goal = np.array([19, 19])
        start_id = rm.add_node(start, is_start=True)
        goal_id = rm.add_node(goal, is_start=False)

        with self.assertRaises(nx.NetworkXNoPath, msg="A* found a path"):
            astar = search.ASTARPlanner(rm)
            _ = astar.Plan(start_id, goal_id)
        
        batch_size = 100
        rrt = search.RRTPlanner(problem, permissible_region, batch_size=batch_size)
        rrt_thread = threading.Thread(target=rrt.Plan, args=(start, goal), daemon=True)
        rrt_thread.start()
        rrt_thread.join(10)
        assert not rrt_thread.is_alive(), "RRT failed to find a path in less than 10 seconds"
        

if __name__ == "__main__":
    rosunit.unitrun("planning", "test_tradeoff", TestTradeoff)
