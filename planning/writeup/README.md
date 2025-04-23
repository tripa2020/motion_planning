# Project 4: Planning [![tests](../../../badges/submit-proj4/pipeline.svg)](../../../pipelines/submit-proj4/latest)

Replace this with your own writeup! Please place all figures and answer all questions in this directory.

**Q1.2** A* Shortest Path Figure: ...
hw4_planning/planning/writeup/run_astar.png
**Q2.1** RRT Path Figure: ...
hw4_planning/planning/writeup/RRT.png
**Q3**

A* beats RRT Figure: ...
hw4_planning/planning/writeup/astar_beats_rrt.png
A* is better than RRT because ...
because Astar is more biased towards the goal it is easier for Astar to go though narrow gaps, while rrt while exploring randomly would have to stumble accross the opening
RRT beats A* Figure: ...

hw4_planning/planning/writeup/rrt_beats_astar.png
RRT is better than A* because ...
it is better at handles random obstacles that are in the way, while astar will get held up on random obstacle between the start and end states

**Q5** A* heuristic design decisions: ...

the sum of the abs angular difference are always non negative, and it represents tha minimal possible cost to reach the goal

**Q6.1**
path length = .45

**Q6.2**

path length = .63

computing the manhattan time is done when the total cost is the sum of the respective cost in each dimensino 