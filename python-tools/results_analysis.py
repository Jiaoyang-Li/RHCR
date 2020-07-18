#!/usr/bin/env python3
from read_file import read_tasks_file, read_paths_file, read_solver_file, read_map_file
from plot_figure import plot_throughput, plot_runtime, plot_idle_drive_units, \
    plot_radar_chart, plot_traffic, set_box_color, plot_induct_stations, draw_animation
from re import split
import argparse
import pathlib
import logging
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as st
import os.path


def generate_agent_file(timestep, agent_fname, path_fanme, task_fname):
    """
    Take a screenshot at the given timestep.
    Output the current locations as start locations
    and the next goal locations as goal locations.
    The output file can be used as the initial start and goal locations
    for a simulation (rather than generating start and goal locations randomly)
    """
    if os.path.exists(path_fanme) is False:
        return

    starts = []
    with open(path_fanme, "r") as f:
        f.readline()
        for line in f.readlines():
            data = split(";", line)[:-1]
            for state in data:
                state = split(",", state)
                loc = int(state[0])
                orient = int(state[1])
                time = int(state[2])
                if time == timestep:
                    starts.append([loc, orient])
                    break

    goals = []
    with open(task_fname, "r") as f:
        f.readline()  # skip number of drives
        for line in f.readlines():
            data = split(";", line)[:-1]
            for task in data:
                task = split(",", task)
                loc = int(task[0])
                time = int(task[1])
                if time > timestep:
                    goals.append(loc)
                    break
            if time <= timestep:
                goals.append(loc)

    with open(agent_fname, "w") as f:
        N = len(goals)
        f.write(str(N)+"\n")
        for i in range(N):
            f.write("{},{},{}\n".format(starts[i][0], starts[i][1], goals[i]))


def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    h = st.sem(a) * st.t.ppf((1 + confidence) / 2., len(a) - 1)
    return h


if __name__ == "__main__":

    colors = ["blue", "orange", "green", "purple", "cyan"]
    markers = [">", "d", "o", "*", "P"]
    fontsize = 15
    simulation_time = 5000
    time_limit = 60  # unit: seconds

    # read map image file, used as the backgound of some plots and videos.
    types, stations, coordinates = read_map_file("../maps/sorting_map.grid")

    # plot runtime and throughput as time goes by
    '''fname = "../exp/PBS/PBS_00drives_w=10"
    num_drive, runtime, node, timestep, success_rate, additional_cost, subopt, cost, bound, length, window \
        = read_solver_file(fname, simulation_time)
    plot_runtime(timestep, runtime, node, time_limit)
    num_drive, throughput, extra_cost, extra_costs_percentage, station_utility = read_tasks_file(fname, simulation_time)
    plot_throughput(throughput, 100)
    exit(0)'''

    # make animations
    '''fname = "../exp/kiva-ECBS/ECBS1.5_180drives_w=10" # "../exp/PBS/PBS_700drives_w=10"
    paths, orientations = read_rotate_paths_file(fname, simulation_time)
    num_drives, throughput, extra_cost, extra_costs_percentage, station_utility = read_tasks_file(fname, simulation_time)
    draw_animation(paths, throughput, orientations, "../maps/kiva_map.png", 46, 33, True)
    exit(0)'''
    '''fname = "../exp/PBS/PBS_800drives_w=10" # "../exp/LRA/LRA_800drives_traveltime"
    paths, orientations = read_rotate_paths_file(fname, simulation_time)
    num_drives, throughput, extra_cost, extra_costs_percentage, station_utility = read_tasks_file(fname, simulation_time)
    draw_animation(paths, throughput, "../maps/sorting_map.png", 37, 77, False)
    exit(0)'''

    # Write the start and goal locations at a given timestep to a file.
    '''fname = "../mid-report/1500drives_Robust_Rotate_WH=10_NoDeadLocks_NoIdle"
    output = "../maps/DEN5_unweighted_directed_rotation=1_1500.agents"
    generate_agent_file(300, output, fname + "/paths.txt", fname + "/tasks.txt")
    exit(0)'''

    # Plot the usage of every induct station
    '''fname = "../exp-0709/1700drives_Robust_Rotate_WH=10_NoReusePaths"
    num_drive, throughput, extra_cost, extra_costs_percentage, station_utility = read_tasks_file(fname, simulation_time)
    induct_stations = []
    for i in range(len(types)):
        if types[i] == "Induct":
            induct_stations.append([i, 0])
            if str(i) in station_utility:
                induct_stations[-1][1] = station_utility[str(i)] # * 2 / 10000
    plt.plot(np.transpose(induct_stations)[1], '.')
    plt.plot([0, len(np.transpose(induct_stations)[1])],[np.mean(np.transpose(induct_stations)[1]), np.mean(np.transpose(induct_stations)[1])], '-')
    plt.xticks(range(len(induct_stations)), [])
    plt.xlabel('Induct station')
    plt.ylabel('Packages')
    plt.show()
    plot_induct_stations(induct_stations, "kmaps/DEN5.png")
    exit(0)'''

    # Plot the traffic, i.e., the usage of every fiducial.
    '''fname = "../exp-0709/1500drives_Robust_Rotate_WH=10_NoReusePaths"
    paths, orientations = read_rotate_paths_file(fname, simulation_time)
    plot_traffic(paths, orientations)
    exit(0)'''

    # Compare the results of different algorithm for different number of drives
    # The followings are to get th paths to the files we are interested in
    # A different naming rules for files may require different codes

    drives = range(100, 200, 100)

    '''folder_name = "../exp/LRA/LRA_"  # the folder that saves all the results
    algo_fnames = ["", "_traveltime"]
    algo_names = ["", "_traveltime"]
    offsets = [-5, 5]'''

    '''# folder_name = "../exp/ECBS/ECBS1.1_"  # the folder that saves all the results
    # folder_name = "../exp/WHCA/WHCA_"  # the folder that saves all the results
    # folder_name = "../exp/CBS/CBS_"  # the folder that saves all the results
    algo_fnames = ["_w=5", "_w=1000"]
    algo_names = ["w=5", "w=inf"]
    offsets = [-5, 5]'''

    '''folder_name = "../exp/PBS/PBS_"  # the folder that saves all the results
    algo_fnames = ["_w=5", "_w=10", "_w=20", "_w=1000"]  #"_w=10_h=10", "_w=20_h=20",
    algo_names = ["w=5", "w=10", "_w=20", "w=inf"]  # "w=10_h=10", "w=20_h=20",
    offsets = [-15, -5, 0,  5, 15]'''

    folder_name = "../exp/PBS/PBS-"  # the folder that saves all the results
    # algo_fnames = ["-weighted2_sorting-w=10", "-weighted5_sorting-w=10", "-weighted10_sorting-w=10"]  #"_w=10_h=10", "_w=20_h=20",
    # algo_names = ["weighted2", "weighted5", "weighted10"]  # "w=10_h=10", "w=20_h=20",
    # algo_fnames = ["-well_formed_e=1-w=10", "-well_formed_e=2-w=10", "-well_formed_e=5-w=10",
    #               "-well_formed_e=10-w=10", "-well_formed_e=inf-w=10", ]  # "_w=10_h=10", "_w=20_h=20",
    algo_fnames = ["-empty_e=1-w=10-h=1", "-empty_e=2-w=10-h=1", "-empty_e=5-w=10-h=1",
                 "-empty_e=10-w=10-h=1", "-empty_e=inf-w=10-h=1", ]  # "_w=10_h=10", "_w=20_h=20",
    algo_names = ["e=1", "e=2", "e=5", "e=10", "e=inf"]  # "w=10_h=10", "w=20_h=20",
    offsets = [-15, -5, 0,  5, 15]
    drives = range(5, 16, 1)

    '''folder_name = "../exp/kiva-ECBS/ECBS1.5_"  # the folder that saves all the results
    algo_fnames = ["_w=5", "_w=5_potential=0.5", "_w=5_potential=0.2", "_w=5_potential=0", "_w=10"]
    algo_names = algo_fnames # ["window", "dummy paths", "hold endpoints"]
    offsets = [-10, -5, 0, 5, 10]
    drives = range(60, 192, 20)'''

    '''#folder_name = "../exp/kiva-PBS/PBS_"  # the folder that saves all the results
    folder_name = "../exp/kiva-ECBS/ECBS1.1_"  # the folder that saves all the results
    algo_fnames = ["_w=10", "_dummy-paths", "_hold-endpoints"]
    algo_names = ["window", "dummy paths", "hold endpoints"]
    offsets = [-10, 0, 10]
    drives = range(60, 192, 40)'''

    online = False

    # read solver files
    num_drives = []
    success_rates = []
    runtimes = []
    avg_runtimes = []
    std_runtimes = []
    last_timesteps = []
    windows = []
    for i in range(len(algo_fnames)):
        print(algo_names[i])
        num_drives.append([])
        success_rates.append([])
        runtimes.append([])
        avg_runtimes.append([])
        std_runtimes.append([])
        last_timesteps.append([])
        windows.append([])
        for k in drives:
            fname = folder_name + str(k) + "drives" + algo_fnames[i]
            if os.path.exists(fname + "/solver.csv") is False:
                continue
            num_drive, runtime, node, timestep, success_rate, additional_cost, subopt, cost, bound, length, window \
                = read_solver_file(fname, simulation_time)
            num_drives[-1].append(num_drive)
            success_rates[-1].append(success_rate)
            runtimes[-1].append(runtime)
            avg_runtimes[-1].append(np.mean(runtime))
            std_runtimes[-1].append(mean_confidence_interval(runtime))
            last_timesteps[-1].append(timestep[-1])
            windows[-1].append(np.mean(window))
        np.set_printoptions(precision=4, floatmode="fixed")
        print("Success rates:                      {}".format(np.array(success_rates[i])))

        print("CPU time (seconds):", end=" ")
        # for avg, std in avg_runtimes[i], std_runtimes[i]:
        for k in range(len(avg_runtimes[i])):
            print("& ${}\pm{}$".format(round(avg_runtimes[i][k], 2), round(std_runtimes[i][k], 2)), end=" ")
        print("\\\\")
        # print("CPU time (seconds):                 {}".format(np.array(avg_runtimes[i])))
        np.set_printoptions(precision=3, floatmode="fixed")
        # print("CPU time std:                       {}".format(np.array(std_runtimes[i])))
        print("Last timestep:                      {}".format(np.array(last_timesteps[i])))
        print("window:                             {}".format(np.array(windows[i])))
    # plot success rates
    for i in range(len(algo_fnames)):
        plt.plot(num_drives[i], success_rates[i], linestyle="-", color=colors[i], marker=markers[i],
                 label=algo_names[i])
    plt.ylim([-0.05, 1.05])
    plt.legend(loc="lower left", fontsize=fontsize)
    plt.xticks(fontsize=fontsize)
    plt.yticks(fontsize=fontsize)
    plt.xlabel("Drives", fontsize=fontsize)
    plt.ylabel("Success rate", fontsize=fontsize)
    plt.show()

    # plot CPU time
    for i in range(len(algo_fnames)):
        length = len(avg_runtimes[i])
        for j in range(len(last_timesteps[i])):
            if last_timesteps[i][j] < last_timesteps[i][0]:
                length = j
                break
        num_drives[i] = drives[:length]
        plt.plot(num_drives[i], avg_runtimes[i][:length], linestyle="-", color=colors[i], marker=markers[i], label=algo_names[i])
        # plt.errorbar(x, y, e, linestyle='None', color=colors[i], marker=markers[i], label=algo_names[i])
    # plt.xticks(drives, drives)
    plt.legend(loc="upper left", fontsize=fontsize)
    plt.xticks(fontsize=fontsize)
    plt.yticks(fontsize=fontsize)
    plt.xlabel("Drives", fontsize=fontsize)
    plt.ylabel("CPU time (s)", fontsize=fontsize)
    plt.show()

    for i in range(len(algo_fnames)):
        x = np.array(num_drives[i]) + offsets[i]
        if len(runtimes[i]) == 0:
            continue
        bp = plt.boxplot(runtimes[i], positions=x, widths=10, showfliers=False)  # showmeans=True,
        set_box_color(bp, colors[i])

    if online:
        for i in range(len(algo_fnames)):
            avg_throughputs = []
            print(algo_names[i])
            for k in drives:
                fname = folder_name + str(k) + "drives" + algo_fnames[i]
                if os.path.exists(fname + "/paths.txt") is False:
                    continue
                paths = read_paths_file(fname, simulation_time)
                avg_throughput = 0
                for path in paths:
                    if path[-1]['timestep'] < simulation_time:
                        avg_throughput += 1
                avg_throughput /= simulation_time
                avg_throughputs.append(avg_throughput)
            np.set_printoptions(precision=3, floatmode="fixed")
            print("Throughput: ", end=" ")
            # base = [3.1868 , 6.3161 , 9.3644 , 12.4624 , 15.4594 , 18.4030 , 21.2996 , 1, 1, 1] # PBS
            # base = [3.1579 , 6.3051 , 9.3084 , 12.2838 , 15.1960 , 1 ,2, 2, 2, 2] # ECBS
            # for i in range(len(avg_throughputs[-1])):
            #    print("& {} ({}\%)".format(round(avg_throughputs[-1][i], 2), round((avg_throughputs[-1][i]/base[i] - 1) * 100, 2)), end=" ")
            for avg in avg_throughputs:
                print("& {}".format(round(avg, 4)), end=" ")
            print("\\\\")
    else:
        # read task file
        num_drives = []
        throughputs = []
        avg_throughputs = []
        std_throughputs = []
        extra_costs = []
        extra_costs_percentages = []
        station_utilities = []
        for i in range(len(algo_fnames)):
            print(algo_names[i])
            num_drives.append([])
            throughputs.append([])
            avg_throughputs.append([])
            std_throughputs.append([])
            extra_costs.append([])
            extra_costs_percentages.append([])
            station_utilities.append([])
            for k in drives:
                fname = folder_name + str(k) + "drives" + algo_fnames[i]
                if os.path.exists(fname + "/tasks.txt") is False:
                    continue
                num_drive, throughput, extra_cost, extra_costs_percentage, station_utility = read_tasks_file(fname,
                                                                                                             simulation_time)
                num_drives[-1].append(num_drive)
                throughputs[-1].append(throughput)
                avg_throughputs[-1].append(np.mean(np.trim_zeros(throughput)))
                std_throughputs[-1].append(np.std(np.trim_zeros(throughput)))
                extra_costs[-1].append(extra_cost)
                extra_costs_percentages[-1].append(extra_costs_percentage)
                station_utilities[-1].append(station_utility)
            np.set_printoptions(precision=3, floatmode="fixed")
            print("Throughput: ", end=" ")
            # base = [3.1868 , 6.3161 , 9.3644 , 12.4624 , 15.4594 , 18.4030 , 21.2996 , 1, 1, 1] # PBS
            # base = [3.1579 , 6.3051 , 9.3084 , 12.2838 , 15.1960 , 1 ,2, 2, 2, 2] # ECBS
            # for i in range(len(avg_throughputs[-1])):
            #    print("& {} ({}\%)".format(round(avg_throughputs[-1][i], 2), round((avg_throughputs[-1][i]/base[i] - 1) * 100, 2)), end=" ")
            for avg in avg_throughputs[-1]:
                print("& {}".format(round(avg, 4)), end=" ")
            print("\\\\")
            # print("Throughput:                              {}".format(np.array(avg_throughputs[-1])))
            # print("Extra task cost:                         {}".format(np.array(extra_costs[-1])))
            # print("Extra task cost (%):                     {}".format(np.array(extra_costs_percentages[-1])))

        # plot throughput
        for i in range(len(algo_fnames)):
            plt.plot(num_drives[i], avg_throughputs[i], color=colors[i], marker=markers[i], label=algo_names[i])
        plt.legend(loc="lower right", fontsize=fontsize)
        plt.xticks(fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        plt.xlabel("Drives", fontsize=fontsize)
        plt.ylabel("Throughput (tasks/timestep)", fontsize=fontsize)
        plt.show()



        # draw temporary lines and use them to create a legend
        h = [[] for i in range(len(algo_names))]
        for i in range(len(algo_names)):
            h[i], = plt.plot([0, 1], '-', color=colors[i])
        plt.legend(h, algo_names, loc="upper right", fontsize=fontsize)
        for i in range(len(algo_names)):
            h[i].set_visible(False)

        plt.xlim([drives[0] - 100, drives[-1] + 100])
        plt.xticks(drives, drives, fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        plt.xlabel("Drive units", fontsize=fontsize)
        plt.ylabel("CPU time (s)", fontsize=fontsize)
        plt.show()

        exit(0)

        # plot total number of idle drives
        for i in range(len(algo_fnames)):
            print(algo_names[i])
            idle_agents = []
            total_agents = []
            for k in drives:
                # if k == 900 and i == 2:
                #     break
                fname = folder_name + str(k) + "drives_Robust_Rotate" + algo_fnames[i]
                if os.path.exists(fname + "/solvers.csv") is False:
                    continue
                paths, orientations = read_paths_file(fname)
                if paths == []:
                    continue
                idle_agent = 0
                for path in paths:
                    for loc in path:
                        if loc < 0:
                            idle_agent += 1
                idle_agents.append(idle_agent / simulation_time)
                total_agents.append(k)
            plt.plot(total_agents, idle_agents, color=colors[i], marker=markers[i], label=algo_names[i])
        plt.legend(loc="upper right", fontsize=fontsize)
        plt.xticks(fontsize=fontsize)
        plt.yticks(fontsize=fontsize)
        plt.xlabel("Drive units", fontsize=fontsize)
        plt.ylabel("Idle drive units", fontsize=fontsize)
        plt.show()

    exit(0)

