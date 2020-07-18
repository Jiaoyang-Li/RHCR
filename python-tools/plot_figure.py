import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from re import split
import matplotlib.patches as patches


def plot_radar_chart(algo_names, stats):
    labels = np.array(['Success rate', 'Scalability', 'Throughput', 'CPU time'])

    angles = np.linspace(0, 2 * np.pi, len(labels), endpoint=False)
    # close the plot

    angles = np.concatenate((angles, [angles[0]]))
    stats = np.concatenate((stats, [stats[:][0]]))
    fig = plt.figure()
    ax = fig.add_subplot(111, polar=True)
    ax.plot(angles, stats, 'o-', linewidth=2)
    ax.fill(angles, stats, alpha=0.25)
    ax.set_thetagrids(angles * 180 / np.pi, labels)
    plt.legend(algo_names)
    # ax.set_title(algo_names)
    ax.grid(True)
    plt.show()
    return


def plot_induct_stations(induct_stations, map_fname):
    cols = 177
    rows = 56
    plt.figure()  # figsize=(10,1))
    ax = plt.gca()
    im = plt.imread(map_fname)
    xdelta = len(im) / rows
    ydelta = len(im[0]) / cols
    plt.imshow(im)  # , origin='upper')
    # Create a Rectangle patch

    avg = np.mean(np.transpose(induct_stations)[1])

    # Add the patch to the Axes)
    for induct in induct_stations:
        loc = [induct[0] % cols * xdelta + 4, len(im) - 2 - int(induct[0] / cols) * ydelta]
        plt.text(loc[0], loc[1], str(round(induct[1] / avg, 2)))
        if loc[1] + induct[1] / 20 > rows * xdelta:
            loc[1] -= induct[1] / 20
        rect = patches.Rectangle(loc, 10, induct[1] / 20, linewidth=1, edgecolor='r', facecolor='r')
        ax.add_patch(rect)
    plt.show()


def draw_animation(paths, throughput, map_fname, cols, rows, rotate):
    duration = 5
    fig = plt.figure(frameon=False, figsize=(12, 7))
    im = plt.imread(map_fname)
    if rotate:
        xdelta = len(im) / rows #rows
        ydelta = len(im[0]) / cols #cols
    else:
        xdelta = len(im) / cols #rows
        ydelta = len(im[0]) / rows #cols
    drives = len(paths)
    min_path_length = float('inf')
    for path in paths:
        if min_path_length >= len(path):
            min_path_length = len(path) - 1

    plt.imshow(im) #, origin='upper')

    ax = plt.gca()
    locs = np.full(drives, None)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    text = ax.text(0, 1, '', transform=ax.transAxes, fontsize=12)

    for i in range(drives):
        locs[i], = plt.plot([], [], 'o', color='#000000', markersize=5)

    def init():
        for i in range(drives):
            locs[i].set_data([], [])
        return locs

    def get_location(path, t):
        if t < 0:
            t = 0
        if t >= len(path):
            t = len(path) - 1
        loc = path[t]
        # x = int(loc % cols * xdelta) #+ 4
        # y = len(im) - int(int(loc / cols) * ydelta) # - 2
        x = len(im[0]) - int(int(loc / cols) * xdelta) - 10
        y = int(loc % cols * ydelta) + 8
        if rotate:
            return y, x
        else:
            return x, y

    # animation function.  this is called sequentially
    def animate(t):
        timestep = int(t / duration)
        ratio = t / duration - timestep
        for i in range(drives):
            x_from, y_from = get_location(paths[i], timestep)
            x_to, y_to = get_location(paths[i], timestep + 1)
            x = x_from * (1 - ratio) + x_to * ratio
            y = y_from * (1 - ratio) + y_to * ratio
            locs[i].set_data(x, y)
        text.set_text("Timestep {:6}: {:4.0f} tasks finished.".format(timestep, throughput[timestep]))
        if t == 1:
            input("Press any key to start")
        return locs

    anim1 = animation.FuncAnimation(fig, animate, init_func=init, frames=min_path_length * duration, interval=1, repeat=False)
    # plt.xlabel(r'$x(m)$')
    plt.show()
    return


def draw_animation_with_orientation(paths, throughput, orientations, map_fname, cols, rows, rotate):
    duration = 5
    fig = plt.figure(frameon=False, figsize=(12, 7))
    im = plt.imread(map_fname)
    if rotate:
        xdelta = len(im) / rows #rows
        ydelta = len(im[0]) / cols #cols
    else:
        xdelta = len(im) / cols #rows
        ydelta = len(im[0]) / rows #cols
    drives = len(paths)
    min_path_length = float('inf')
    for path in paths:
        if min_path_length >= len(path):
            min_path_length = len(path) - 1

    plt.imshow(im) #, origin='upper')

    ax = plt.gca()
    locs = np.full(drives, None)
    dirs = np.full(drives, None)
    # idles = np.full(drives, None)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    text = ax.text(0, 1, '', transform=ax.transAxes, fontsize=12)

    for i in range(drives):
        locs[i], = plt.plot([], [], 'o', color='orange', markersize=5)
        dirs[i], = plt.plot([], [], '-', color='black', linewidth=0.5)
        #idles[i], = plt.plot([], [], 'o', color='blue', markersize=5)

    def init():
        for i in range(drives):
            locs[i].set_data([], [])
            dirs[i].set_data([], [])
            #idles[i].set_data([],[])
        #text.set_text("")
        return locs, dirs#, idles, text

    def get_location(path, t):
        if t < 0:
            t = 0
        if t >= len(path):
            t = len(path) - 1
        loc = path[t]
        # x = int(loc % cols * xdelta) #+ 4
        # y = len(im) - int(int(loc / cols) * ydelta) # - 2
        x = len(im[0]) - int(int(loc / cols) * xdelta) - 7
        y = int(loc % cols * ydelta) + 8
        if rotate:
            return y, x
        else:
            return x, y

    # animation function.  this is called sequentially
    def animate(t):
        timestep = int(t / duration)
        ratio = t / duration - timestep
        for i in range(drives):
            x_from, y_from = get_location(paths[i], timestep)
            x_to, y_to = get_location(paths[i], timestep + 1)
            x = x_from * (1 - ratio) + x_to * ratio
            y = y_from * (1 - ratio) + y_to * ratio
            locs[i].set_data(x, y)
            dir = orientations[i][timestep]
            if dir == 3:
                dirs[i].set_data([x, x - xdelta / 2], [y, y])
            elif dir == 2:
                dirs[i].set_data([x, x], [y, y + ydelta / 2])
            elif dir == 1:
                dirs[i].set_data([x, x + xdelta / 2], [y, y])
            elif dir == 0:
                dirs[i].set_data([x, x], [y, y - ydelta / 2])
            else:
                dirs[i].set_data([x, x], [y, y])
        text.set_text("Timestep {:6}: {:4.0f} tasks finished.".format(timestep, throughput[timestep]))
        if t == 1:
            input("Press any key to start")
        return locs, dirs#, idles, text

    anim1 = animation.FuncAnimation(fig, animate, init_func=init, frames=min_path_length * duration, interval=1, repeat=False)
    # plt.xlabel(r'$x(m)$')
    plt.show()
    return


def moving_average(interval, window_size):
    window = np.ones(int(window_size))/float(window_size)
    return np.convolve(interval, window, 'valid')


def plot_throughput(task, window_size):
    task = moving_average(np.trim_zeros(task, 'b'), window_size)

    plt.plot(task)
    plt.xlabel("Timestep")
    plt.ylabel("Throughput (tasks / timestep)")
    plt.show()
    # plt.plot(extra_cost / count)
    # plt.xlabel("Timestep")
    # plt.ylabel("Extra cost / timestep)")
    # plt.show()

    # y = moving_average(tasks, 100)
    # plt.plot(y[100:-100])
    return


def plot_traffic(paths, orientations):
    cols = 177
    rows = 56
    map = np.ones(shape=(rows, cols), dtype=int)
    waits = np.zeros(shape=(rows, cols), dtype=int)

    for i in range(len(paths)):
        pres = (-1, -1)
        for j in range(len(paths[0])):
            loc = paths[i][j]
            dir = orientations[i][j]
            x = loc % cols
            y = int(loc / cols)
            map[y][x] += 1
            if pres[0] == loc and pres[1] == dir:
                waits[y][x] += 1
            pres = (loc, dir)

    plt.subplot(2, 1, 1)
    plt.title("Traffic distribution")
    plt.imshow(map, cmap='hot', interpolation='nearest', origin='lower')
    plt.colorbar()
    # plt.clim(0, 10000)
    plt.subplot(2, 1, 2)
    plt.title("Wait actions distribution")
    plt.imshow(waits, cmap='hot', interpolation='nearest', origin='lower')
    plt.colorbar()
    # plt.clim(0, 10000)
    plt.show()


def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)


def plot_runtime(timestep, runtime, node, time_limit):
    runtime = np.clip(runtime, 0, time_limit)
    # plt.subplot(2, 1, 1)
    plt.plot(timestep, runtime, "g.")
    for i in range(len(timestep)):
        if runtime[i] == time_limit:
            plt.plot(timestep[i], runtime[i], "r.")
    # plt.title("PBS runtime")
    plt.xlabel("Timestep")
    plt.ylabel("CPU time (seconds)")
    # plt.subplot(2, 1, 2)
    # plt.plot(timestep, node, ".")
    # plt.title("PBS expanded nodes")
    # plt.xlabel("Timestep")
    # plt.ylabel("Expanded nodes")
    plt.show()


def plot_idle_drive_units(paths):
    max_timestep = float('inf')
    for path in paths:
        max_timestep = min(max_timestep, len(path))
    idle_drives = np.zeros(max_timestep)
    for path in paths:
        for t in range(max_timestep):
            if path[t] < 0:
                idle_drives[t] += 1
    plt.plot(idle_drives)
    plt.xlabel("Timestep")
    plt.ylabel("Idle drives")
    plt.show()