# Lifelong-MAPF-Simulation

The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake: 
- Works for Boost versions below 1.73

```
cmake .
make
```

Then, you are able to run the code:
```
./lifelong -m maps/sorting_e=inf.grid -k 800 --scenario=SORTING --simulation_window=5 --planning_window=10 --simulation_time=250 --output=../exp/single-planning_window=4 --linkage=1
```

You can find details and explanations for all parameters with:
```
./lifelong --help
```
