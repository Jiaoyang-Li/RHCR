# Lifelong-MAPF-Simulation

The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake: 
```
cmake .
make
```

Then, you are able to run the code:
```
./lifelong -m maps/sorting_e=inf.grid -k 800 --scenario=SORTING --simulation_window=5 --planning_window=10 --seed=0
```

You can find details and explanations for all parameters with:
```
./lifelong --help
```
