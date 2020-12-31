# RHCR

Rolling-Horizon Collision Resolution (WHCR) is an effifent algorithm for solving lifelong Multi-Agent Path Finding (MAPF) where we are aksed to plan collision-free paths for a large number of agents that are constanly engaged with new goal locations. WHCR calls a Windowed MAPF solver every h timesteps that resolves collisions only for the next w timesteps (w >= h). More details can be found in our extended abstract at AAMAS 2020 [1] or our full paper at AAAI 2021 [2].

The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake: 
```
cmake .
make
```

Then, you are able to run the code:
```
./lifelong -m maps/sorting_map.grid -k 800 --scenario=SORTING --simulation_window=5 --planning_window=10 --solver=PBS --seed=0
```

- m: the map file 
- k: the number of agents
- scenario: the simulation scenario (each scenario corresponding to a different task assigner). Use KIVA for the fulfillment warehouse scenario and SORTING for the sorting center scenario. 
- simulation_window: the replanning period h
- planning_window: the planning window w
- solver: the windowed MAPF solver (WHCA, ECBS, and PBS)
- seed: the random seed

You can find more details and explanations for all parameters with:
```
./lifelong --help
```

## License
RHCR is released under USC – Research License. See license.md for further details.
 
## References
[1] Jiaoyang Li, Andrew Tinka, Scott Kiesel, Joseph W. Durham, T. K. Satish Kumar and Sven Koenig. Lifelong Multi-Agent Path Finding in Large-Scale Warehouses (extended abstract). In Proceedings of the International Joint Conference on Autonomous Agents and Multiagent Systems (AAMAS), pages 1898-1900, 2020.

[2] Jiaoyang Li, Andrew Tinka, Scott Kiesel, Joseph W. Durham, T. K. Satish Kumar and Sven Koenig. Lifelong Multi-Agent Path Finding in Large-Scale Warehouses. In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), (in print), 2021.
