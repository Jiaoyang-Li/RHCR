import numpy as np
from re import split
#from spatial.fiducial_graph import make_fid_graph, get_compass_direction


def read_map_file(fname):
    with open(fname, "r") as f:
        f.readline()  # skip "Grid size (rows, cols)"
        line = f.readline()
        line = split(",", line[:-1])
        rows = int(line[0])
        cols = int(line[1])
        f.readline()  # skip the headers
        types = []
        stations = []
        coordinates = []
        for line in f.readlines():
            data = split(",", line[:-1])
            types.append(data[1])
            stations.append(data[2])
            coordinates.append([int(data[3]), int(data[4])])
        return types, stations, coordinates


def read_tasks_file(fname, simulation_time):
    station_utility = dict()
    with open(fname + "/tasks.txt", "r") as f:
        drives = int(f.readline()[:-1])
        throughput = np.zeros(simulation_time + 1)
        task_min_cost = 0
        extra_cost = 0
        count = 0
        prev = -1
        for line in f.readlines():
            data = split(";", line)
            for tuple in data:
                if tuple == "" or tuple == "\n":
                    break
                tuple = split(",", tuple)
                loc = tuple[0]
                time = int(tuple[1])
                if time > simulation_time:
                    break
                elif time > 0:
                    throughput[time] += 1
                    if loc in station_utility:
                        station_utility[loc] += 1
                    else:
                        station_utility[loc] = 1
                    if len(tuple) == 3 and tuple[2] != "":
                        task_min_cost += int(tuple[2])
                        extra_cost += time - prev - int(tuple[2])
                        count += 1
                prev = time
        count = max(count, 1)
        task_min_cost = max(task_min_cost, 1)
        return drives, throughput, extra_cost / count, extra_cost / task_min_cost * 100, station_utility


'''def read_paths_file(fname, plot):
    if plot == 0:
        return
    with open(fname, "r") as f:
        drives = f.readline()[:-1]
        cols = 177
        rows = 56
        map = np.zeros(shape=(rows, cols), dtype=int)
        waits = np.zeros(shape=(rows, cols), dtype=int)
        for line in f.readlines():
            data = split(",", line)[:-1]
            for i in range(len(data)):
                loc = data[i]
                x = int(loc) % cols
                y = int(int(loc) / cols)
                map[y][x] += 1
                if i > 0 and data[i - 1] == loc:
                    waits[y][x] += 1
        if plot == 1:
            plt.subplot(2, 1, 1)
            plt.title("Traffic distribution")
            plt.imshow(map, cmap='hot', interpolation='nearest', origin='lower')
            plt.colorbar()
            plt.subplot(2, 1, 2)
            plt.title("Wait actions distribution")
            plt.imshow(waits, cmap='hot', interpolation='nearest', origin='lower')
            plt.colorbar()
            plt.show()
        return'''


def read_paths_file(fname, simulation_time):
    with open(fname + "/paths.txt", "r") as f:
        paths = []
        f.readline()  # skip the number of agents
        for line in f.readlines():
            data = split(";", line)[:-1]
            paths.append([])
            for tuple in data:
                tuple = split(",", tuple)
                if int(tuple[2]) > simulation_time:
                    break
                paths[-1].append({'location': int(tuple[0]), 'orientation': int(tuple[1]), 'timestep': int(tuple[2])})

        return paths


def read_solver_file(fname, simulation_time):
    idx_runtime = 0
    idx_nodes = 1
    idx_cost = 5
    idx_bound = 6
    idx_length = 7
    idx_conflicts = 8
    idx_time = 9
    idx_drives = 10
    idx_window = 12

    num_drive = -1
    runtime = []
    node = []
    cost = []
    bound = []
    length = []
    timestep = []
    additional_cost = 0
    subopt = []
    num_failure = 0
    windows = []

    with open(fname + "/solver.csv", "r") as f:
        # f.readline() # skip the header line
        for line in f.readlines():
            data = split(",|\n| ", line[:-1])
            if num_drive == -1:
                num_drive = int(data[idx_drives])
            elif int(data[idx_time]) > simulation_time:
                break
            if float(data[idx_cost]) < 0:
                num_failure += 1
            if timestep != [] and timestep[-1] == int(data[idx_time]):  # the primary solver fails
                runtime[-1] += float(data[idx_runtime])
                node[-1] += int(data[idx_nodes])
            else:
                runtime.append(float(data[idx_runtime]))
                node.append(int(data[idx_nodes]))
                timestep.append(int(data[idx_time]))
                if len(data) > idx_window and data[idx_window].isnumeric():
                    windows.append(int(data[idx_window]))
            if float(data[idx_cost]) > 0:
                cost.append(float(data[idx_cost]))
                bound.append(float(data[idx_bound]))
                length.append(float(data[idx_length]))
                additional_cost += cost[-1] - bound[-1]
                subopt.append((cost[-1] - bound[-1]) * 100.0 / bound[-1])

        return num_drive, runtime, node, timestep, 1 - num_failure / len(timestep), additional_cost, subopt, cost, bound, length, windows


def read_results_file(fname):
    idx_runtime = 0
    idx_nodes = 1
    idx_cost = 5
    idx_bound = 7
    idx_drives = 9

    num_drives = []
    runtime = []
    nodes = []
    cost = []
    bound = []
    with open(fname, "r") as f:
        # f.readline() # skip the header line
        for line in f.readlines():
            data = split(",|\n| ", line)
            if len(num_drives) == 0 or int(data[idx_drives]) != num_drives[-1]:
                if len(num_drives) != 0:
                    runtime[-1] /= count
                    nodes[-1] /= count
                    bound[-1] /= count # (cost[-1] / bound[-1] - 1) * 100
                    cost[-1] /= count # * num_drives[-1]
                num_drives.append(int(data[idx_drives]))
                runtime.append(float(data[idx_runtime]))
                nodes.append(int(data[idx_nodes]))
                cost.append(float(data[idx_cost]))
                bound.append(float(data[idx_bound]))
                count = 1
            # elif int(data[idx_cost]) < 0:
            #     continue;
            else:
                runtime[-1] += float(data[idx_runtime])
                nodes[-1] += int(data[idx_nodes])
                cost[-1] += float(data[idx_cost])
                bound[-1] += float(data[idx_bound])
                count += 1
        runtime[-1] /= count
        nodes[-1] /= count
        bound[-1] /= count # (cost[-1] / bound[-1] - 1) * 100
        cost[-1] /= count # * num_drives[-1]

        print("#drives  runtime  suboptimal      nodes   sum-of-cost delta-cost path-cost")
        for i in range(len(num_drives)):
            print("{:4}     {:5.1f}     {:6.03f}        {:5.0f}     {:7.0f}     {:4.0f}         {:3.0f}\n".format( \
                num_drives[i], runtime[i],  (cost[i] - bound[i]) * 100 / bound[i], \
                nodes[i], cost[i], cost[i] - bound[i], cost[i] / num_drives[i]))