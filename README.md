# Automated-planning-Robot-solver
This project focuses on the development and empirical evaluation of an automated planning system for a domain consisting of a robot navigating a grid-based maze containing walls, boxes, keys, doors and labelled drop zones, with the objective of delivering each box to its corresponding drop zone while respecting action preconditions and constraints.

**Introduction**

This project focuses on the development and empirical evaluation of an automated planning system for a domain. The domain consists of a robot navigating a grid-based maze containing walls, boxes, keys, doors, and labelled drop zones, with the objective of delivering each box to its corresponding drop zone while respecting action preconditions and constraints. The work combines classical state-space search algorithms, heuristic guidance, plan validation, declarative domain modelling using PDDL, and a graphical user interface for visualization and analysis. The project aims not only to find valid plans but also to study the trade-offs between different search strategies and heuristics in terms of efficiency, solution quality, and completeness.

**Implementation**

**Problem-specific Solver (Python)**

The core of the system is a forward state-space planner implemented in Python. A central design decision was to define an appropriate state representation that could support efficient expression and update of information.

The environment is represented as a Maze class, which parses a .csv file into a structured representation of the initial maze state. Positions of walls are stored as a Boolean grid, noting the positions where the walls are present. In this class, the positions and labels of boxes, keys, drop zones, and doors are also saved as tuples. Lookup dictionaries were created for each of these elements for constant-time queries during planning. This feature was added because it significantly improves performance in terms of computation compared to linear time complexity of lookups in Python lists.

Each state is represented in a State class, and encodes the robot’s position, the identity of a currently carried box if any, the positions and labels of all remaining boxes, the set of keys already collected, and the positions and labels of keys still present in the maze. All variables are stored using immutable data structures, primarily frozensets, which ensures that states are hashable  and can be reliably stored in closed lists and cost maps. In development, we found that each state needs to be hashable to be able to correctly compare different states, thus avoiding visiting the same states many times over. 

The successor function is implemented as a deterministic transition function that generates all applicable actions for a given state. These actions include movement in the four cardinal directions, subject to walls and locked doors, moving through doors if a corresponding key is owned, lifting a box when the robot is located on the same cell and is not already carrying one, dropping a box when the robot is on the appropriate drop zone, and collecting a key when one is present at the robot’s location. Each action is checked against explicit preconditions derived from the domain rules, and applying an action produces a new immutable successor state.

Several classical search strategies were implemented on top of this state-transition model. Breadth-first search explores the state space in layers of increasing depth and guarantees optimality with respect to plan length, assuming uniform action costs. A* search combines accumulated path cost with heuristic estimates to guide exploration and, when used with an admissible heuristic, guarantees both completeness and optimality. Greedy best-first search relies solely on heuristic values and prioritises states that appear closest to the goal, sacrificing optimality and completeness in exchange for reduced search effort. Enforced hill climbing attempts to greedily improve heuristic values and performs local breadth-first searches when no immediate improvement is possible, a strategy commonly used in satisficing planners.
Two main heuristic approaches were implemented. The first is a domain-specific, distance-based heuristic that estimates remaining cost using Euclidean or Manhattan distances between the robot, boxes, drop zones, and relevant keys. This heuristic is informative because it captures spatial structure and loosely approximates the effort required to move objects to their goals. However, it is not admissible, as it can double-count movement costs, treats goals as independent, and ignores any shared actions between the paths to each goal [1]. This limitation leads to algorithms such as A star to at times discard optimal routes to the solution and find solutions which are longer than optimal (which breaks the A star’s optimality guarantee). 
The second heuristic is an approximation of the Fast-Forward heuristic, commonly referred to as hFF. This heuristic constructs a relaxed version of the planning problem by ignoring delete effects, meaning that once a fact becomes true it remains true for the remainder of the relaxed plan. A relaxed planning graph is built forward from the current state, and the heuristic value is computed as the number of actions in the relaxed plan needed to achieve all goals. The hFF heuristic is informative because it reasons explicitly about goal achievement rather than geometric distance alone. It is not admissible, since delete relaxation underestimates the true cost of achieving the goal, however it should provide good guidance in practice. 

Finally, a dedicated plan validation step was added after plan generation. Validation starts from the initial state and sequentially applies each action in the returned plan. For each step, the system checks whether the action is applicable in the current state, whether the successor state matches the domain’s transition rules, and whether the final state satisfies the goal conditions.

A 60 second timeout timer was also added to the solver function to prevent excessive planning times. The design rationale behind this decision was that, in practice, we would need a planner to come up with a solution in a reasonably short amount of time. Any plan that needs a longer time to compute is too slow to be practically useful.
In addition to the mentioned, a Graphical User Interface was developed for the purpose of easy visualisation of different search strategies, as well as to allow interactive and quick experimentation. The GUI supports loading maze files, selecting algorithms and heuristics, running the planner, displaying performance statistics, visualising the maze and animating the final plan if one exists, and adjusting the speed of the animation. The visualisation is decoupled from the planning process. This architectural decision was necessary because rendering every expanded state drastically slowed down the planner and caused consistent timeouts on all search strategies. The final design allows planning to run at full speed while animating only the solution afterward.

**Generic Solver (PDDL)**
In addition to the procedural planner, the domain was also encoded declaratively using the Planning Domain Definition Language. The PDDL domain file defines predicates for the robot’s position, box locations, carried objects, key ownership, door states, and drop zone satisfaction, along with action schemas corresponding to movement, lifting, dropping, and key collection. Each maze instance is translated into a PDDL problem file by mapping grid cells to location objects, instantiating initial predicates based on the maze layout, and specifying goal predicates that require all boxes to be delivered. The use of PDDL provides a clean, declarative specification of the domain and enables comparison with a generic online PDDL-based planner.
Walls and grid connectivity are treated as static features and therefore are not part of the dynamic state. This design choice reduces the state space and avoids redundant information being copied between states.
For each state, the planner checks the preconditions of every action (movement, key pickup, door traversal, box lifting and box dropping). If the preconditions are satisfied, the action is applied to generate a successor state, updating the robot’s position, carried object, or object locations accordingly.
To prevent loops, each generated state is stored in a visited set using a canonical representation of the robot position, carried box (if any), box locations, and owned keys.
Search strategies: For the generic planning component, domain-independent planners were used to solve the PDDL-encoded problems. In contrast to the domain-specific solver developed in Part 1, these planners automatically derive heuristics and search strategies from the declarative domain description.
Two planners were used:
BFWS is a heuristic search strategy that combines best-first search with width-based pruning. Instead of prioritising nodes solely based on cost or heuristic estimates, BFWS uses novelty measures to guide the search. A state is considered novel if it makes a previously unseen combination of fluents true. As a result, BFWS is particularly effective in domains where progress toward the goal can be characterised by the introduction of new facts, such as acquiring keys or delivering boxes.
In this project, BFWS was used via the FF parser, which automatically extracts heuristic information from the PDDL domain. The planner requires no manual heuristic specification and relies entirely on the declarative domain model to guide the search.
ENHSP is a heuristic search planner designed to handle expressive PDDL domains, including those involving numeric fluents and complex action constraints. Although the current domain does not make use of numeric fluents, ENHSP still provides powerful heuristic guidance through relaxed planning graph analysis. ENHSP performs a forward heuristic search, expanding states based on estimated distance to the goal. Its heuristics consider the relaxed reachability of goals while ignoring certain constraints, allowing the planner to efficiently estimate progress even in domains with doors, keys, and object dependencies.
The planner is particularly well-suited to problems where achieving goals requires satisfying prerequisite conditions, such as acquiring keys before traversing doors. This makes it appropriate for the warehouse maze domain used in this project.

**Evaluation**
	
The performance of the implemented search strategies was evaluated empirically using several metrics, including number, the number of expanded states, the length of the plan found, and whether the algorithm found a valid plan. The results were recorded and analysed across different algorithm and heuristic combinations. The same mazes were also solved using an online generic PDDL-based planner, and comparable metrics were recorded where applicable, excluding runtime due to the differences in hardware and inconsistencies inherent in using this metric as a variable in algorithm analysis. 

**Python Planning Algorithms**
	The performance of implemented solutions across all heuristics in terms of expanded states can be found in Figure 1.

Figure 1 - Average states expanded by algorithm - all heuristics

Breadth-first search exhibited the expected behaviour. It was the only uninformed search algorithm out of the four implemented algorithms, meaning that it did not make use of any heuristics when searching for a plan. It consistently found optimal solutions in terms of plan length but expanded an extremely large number of states, with an average of 18227.6 expanded states per maze. This makes BFS by far the least efficient algorithm when considering this metric. These findings align with the established body of knowledge on this search strategy. This search method managed to find a solution to every maze. For all mazes, the number of generated states by this search method was larger than the number of explored states by 1. Plan lengths produced by this algorithm were used as the golden standard for optimality when evaluating the plan length of other algorithms.

Figure 2 - States expanded per maze - Breadth-First Search

A* search achieved a good balance between efficiency and solution quality. It expanded an average of 6183.86 states across both heuristics. When combined with the distance-based heuristic, it expanded significantly fewer states than BFS while still producing valid plans of short length. Although the distance-based heuristic was not admissible, the solutions found by A* were usually optimal or close to optimal. A* was able to find solutions to all mazes when paired with distance-based heuristic. Average states expanded by A* divided by heuristic can be seen in Figures 3, 4, and  5. When A* was combined with H-ff heuristic, its performance in terms of expanded states became worse, and it was unable to find solutions to more complex mazes because the computation would exceed the 60 second timeout period. This algorithm was the only one where the difference between the number of generated and expanded states was higher than 1, with the average difference of 78.2 states. This algorithm was also the only one where the h-FF heuristic marginally improved performance in terms of states expanded.

Figure 3 - Average states expanded by heuristic - A*

Figure 4 - States Expanded by maze - distance heuristic (A*)

Figure 5 - States Expanded by maze - H-ff heuristic (A*)

Greedy best-first search showed mixed performance. In cases where the heuristic provided clear guidance, it found solutions quickly with very few expanded states, with an average states expanded of 4409.94 across both heuristics, which is the lowest average among the implemented algorithms. However, it was highly sensitive to heuristic inaccuracies and often found solutions which were far from optimal. It failed to find a solution to one maze by the way of timeout, each time using the H-ff heuristic method.


Figure 6 - Average states expanded by heuristic - Greedy Best-First search


Figure 7 - States Expanded per maze - Greedy Best-First Search (Distance heuristic)


Figure 8 - States Expanded per maze - Greedy best-First Search (H-ff)

Enforced hill climbing performed well, but only under favourable conditions. It was one of the more efficient algorithms, having an average states expanded of 4971.12, however this efficiency came at the cost of completeness and optimality. With the hFF heuristic, it sometimes found solutions efficiently, but at times terminated prematurely due to heuristic plateaus or local minima. Furthermore, for the more complex mazes, its performance became significantly worse. This behaviour is consistent with theoretical expectations, as enforced hill climbing is neither complete nor optimal and relies heavily on the heuristic’s ability to provide a strictly improving gradient toward the goal [1]. With the distance-based heuristic, it was able to consistently and efficiently find solutions to most mazes. It was the only search algorithm which couldn’t find solutions to all mazes using the distance-based heuristics, failing to find a solution to mazes 7, 8 and 10 specifically.


Figure 9 - Average states expanded by heuristic - Enforced Hill Climbing



Figure 10 - States Expanded per maze - Enforced Hill Climb (Distance Heuristic)



Figure 11 - States expanded per maze - Enforced Hill Climbing (H-ff)

These results suggest that the easiest maze to solve was Maze 1, since all algorithms and heuristics were able to solve it efficiently. On the other hand, the most difficult maze to solve was Maze 7, since it needed the most expanded states for all algorithms with all heuristics, and some heuristics were unable to help the algorithm solve this maze before the timeout period. Enforced hill climbing was the only algorithm that was able to solve Maze 7 using the H-ff heuristic in the required amount of time.

When it comes to heuristics implemented, the distance-based heuristic was much more performant in terms of expanded states, as can be seen in Figure 12. The H-FF heuristic was computationally more expensive to run and expanded more states, leading to exceedingly higher runtimes and frequent timeouts. The number of expanded states and general inefficiency of our implementation is in contrast with previously established knowledge on this heuristic function [2], leading us to conclude that there were implementation errors made while developing this heuristic. However, at the time of writing, these errors have not been discovered and patched.


Figure 12 - Average states expanded by heuristic - All algorithms


**PDDL Planners**

The comparison with the online PDDL planner showed that generic planners are highly competitive in terms of the number of expanded states and plan quality, particularly when using sophisticated heuristics and pruning techniques. However, the custom planner provided greater transparency and control, allowing detailed analysis of algorithmic behaviour.
This section analyses the performance of the domain-independent planners BFWS (FF-parser version) and ENHSP across all ten mazes. The comparison focuses on plan quality and search effort, using the number of generated states, number of expanded states, and plan length as the primary metrics. Although planning time is reported, it is discussed qualitatively rather than quantitatively due to hardware differences.

Figure 13 - plan length between each Maze using BFWS & ENHSP 
Across all mazes, BFWS consistently produced shorter plans than ENHSP. Both planners achieved optimal or near-optimal solutions with identical plan lengths. However, as maze complexity increased, ENHSP tended to generate significantly longer plans.
For example, in Maze 7 and Maze 8, BFWS produced plans of length 19, whereas ENHSP produced plans of length 31. This indicates that BFWS is more effective at guiding the search toward goal-relevant actions, whereas ENHSP sacrifices solution quality in favour of heuristic robustness.
This behaviour is expected, as BFWS explicitly prioritises novelty and goal-relevant fluents, while ENHSP relies on relaxed planning heuristics that do not guarantee optimality.

Figure 14 - number of states generated between each Maze using BFWS & ENHSP
In the number of States generated the graphs comparing the number of generated states show that ENHSP generally evaluates fewer states than BFWS across all mazes. This is particularly noticeable in the more complex mazes, such as Maze 7 to Maze 10, where ENHSP generates approximately 53 states compared to BFWS’s 70–80 states.
This demonstrates that ENHSP’s heuristic guidance is effective at pruning large portions of the search space early, even when the resulting plan is not optimal. BFWS, while still efficient, explores a broader portion of the state space to maintain better solution quality.
        
Figure 15 - number of states expanded between each Maze using BFWS & ENHSP          
The number of states Expanded have a similar trend and its observed in the number of expanded states. ENHSP consistently expands fewer states than BFWS, indicating that it commits earlier to promising search paths. BFWS, on the other hand, expands more states as it evaluates novelty across multiple dimensions of the state space.
Despite expanding more states, BFWS benefits from its width-based pruning mechanism, which prevents uncontrolled state explosion and maintains reasonable scalability across larger mazes.

Figure 16 - time taken between each Maze using BFWS & ENHSP
The planning time increases gradually with maze complexity for both planners. BFWS exhibits extremely low planning times across all mazes, remaining in the sub-millisecond range. ENHSP requires noticeably longer planning times, typically between 0.03 and 0.04 seconds.
These timing differences are included for completeness but are not directly compared, as the online planners execute on different hardware and use different internal optimisations.
	These results demonstrate that domain - independent planners can effectively solve the warehouse planning problem without hand crafted heuristics. However, their behaviour varies significantly depending on the underlying search strategy, reinforcing the importance of planner selection when balancing solution quality against search effort. 


**Conclusion**
This project demonstrates the practical challenges and trade-offs involved in developing an automated planning program. The choice of state representation and successor function directly affects both correctness and performance, while heuristic design plays a central role in guiding the search algorithm towards an acceptable solution. Although admissible heuristics provide theoretical guarantees, non-admissible heuristics such as our distance-based heuristic often yield good practical performance at the cost of optimality. The integration of PDDL modelling highlights the value of declarative domain descriptions and enables meaningful comparison with established planning systems. This project illustrates how theoretical planning concepts translate into real-world implementations and how empirical evaluation is essential for understanding their strengths and limitations.


References

[1] A. Coles and A. Smith, 2007. “Heuristics for Forward-Chaining Planning”, Online. At: https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume28/coles07a-html/node4.html 


[2] J. Hoffman, 2001, “The Fast-Forward Planning System”, AI Magazine Vol. 22, No 3 

