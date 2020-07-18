#!/usr/bin/env python3
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from re import split


class FlowerMap:
    def __init__(self, fname):
        with open(fname, "r") as f:
            line = f.readline()  # size
            self.x_max = int(split(" ", line)[1]) + 2
            self.y_max = self.x_max
            line = f.readline()  # removes
            self.num_obstacles = int(split(" ", line)[1])
            f.readline()  # skip "D"
            f.readline()  # skip "N"
            f.readline()  # skip "R"
            f.readline()  # skip "T"
            f.readline()  # skip "Q"
            f.readline()  # skip demand
            f.readline()  # skip theta_1
            f.readline()  # skip theta_2
            f.readline()  # skip theta_d
            line = f.readline()  # D_locations
            flowers = split(" ", line)[1:-1]
            self.flowers = []
            for flower in flowers:
                self.flowers.append(int(flower) - 1)
            line = f.readline()  # N_location
            self.entrance = int(split(" ", line)[1]) - 1
            f.readline()  # skip "R_locations"
            line = f.readline()  # remove_locations
            obstacles = split(" ", line)[1:-1]
            self.obstacles = []
            for obstacle in obstacles:
                self.obstacles.append(self.get_coordinate(int(obstacle) - 1))
            for x in range(self.x_max):
                self.obstacles.append((x, 0))
                self.obstacles.append((x, self.y_max - 1))
            for y in range(1, self.y_max - 1):
                self.obstacles.append((0, y))
                self.obstacles.append((self.x_max - 1, y))

    def get_coordinate(self, id):
        return int(id / (self.y_max - 2)) + 1, id % (self.y_max - 2) + 1


def read_paths(fname):
    with open(fname + "\\paths.txt", "r") as f:
        paths = []
        f.readline()  # skip the number of agents
        for line in f.readlines():
            data = split(";", line)[:-1]
            paths.append([])
            for s in data:
                s = split(",", s)
                paths[-1].append(int(s[0]))
        return paths


def read_tasks(fname):
    with open(fname + "/tasks.txt", "r") as f:
        bees = int(f.readline()[:-1])
        tasks = []
        for line in f.readlines():
            data = split(";", line)[:-1]
            for s in data[1:]:
                s = split(",", s)
                tasks.append((int(s[0]), int(s[1])))
        return tasks


# Colors = ['green', 'blue', 'orange']
Colors = ['#E74C3C', '#F1C40F',
          '#A569BD', '#5DADE2', '#E67E22',
          '#95A5A6', '#1E8449',
          'blue', 'yellow', 'cyan', 'orange', 'green', '#27AE60']


class Animation:
    def __init__(self, my_map, paths, tasks):
        self.delta_t = 5
        self.my_map = my_map
        self.T = len(paths[0]) - 1
        self.paths = []
        if paths:
            for path in paths:
                self.paths.append([])
                for loc in path:
                    self.paths[-1].append(my_map.get_coordinate(loc))
        aspect = self.my_map.x_max / self.my_map.y_max

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        # create boundary patch

        x_min = -0.5
        y_min = -0.5
        x_max = self.my_map.x_max - 0.5
        y_max = self.my_map.y_max - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        # flowers
        self.flowers = []
        for flower in self.my_map.flowers:
            loc = self.my_map.get_coordinate(flower)
            self.patches.append(Rectangle((loc[0] - 0.5, loc[1] - 0.5), 1, 1, facecolor='yellow', edgecolor='none'))

        # tasks
        self.tasks = []
        for task in tasks:
            loc = my_map.get_coordinate(task[0])
            t = task[1]
            self.tasks.append((Rectangle((loc[0] - 0.5, loc[1] - 0.5), 1, 1, facecolor='orange', edgecolor='none'), t))
            self.tasks[-1][0].original_face_color = 'orange'
            self.patches.append(self.tasks[-1][0])
        entrance = self.my_map.get_coordinate(self.my_map.entrance)
        self.patches.append(Rectangle((entrance[0] - 0.5, entrance[1] - 0.5), 1, 1, facecolor='green'))  # entrance

        # obstacles
        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='grey'))
        for obstacle in self.my_map.obstacles:
            self.patches.append(Rectangle((obstacle[0] - 0.5, obstacle[1] - 0.5), 1, 1, facecolor='gray', edgecolor='grey'))

        # create agents:
        for i in range(len(self.paths)):
            name = str(i)
            self.agents[i] = Circle(self.paths[i][0], 0.3, facecolor=Colors[i % len(Colors)],
                                    edgecolor='black')
            self.agents[i].original_face_color = Colors[i % len(Colors)]
            self.patches.append(self.agents[i])
            self.agent_names[i] = self.ax.text(self.paths[i][0][0], self.paths[i][0][1] + 0.25, name)
            self.agent_names[i].set_horizontalalignment('center')
            self.agent_names[i].set_verticalalignment('center')
            self.artists.append(self.agent_names[i])

        self.artists.append(self.ax.text(0, self.my_map.x_max - 1, 'Timestep 0'))
        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 1) * self.delta_t,
                                                 interval=20,
                                                 blit=True)

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        # clock = [self.ax.text(0, len(self.my_map[0]) - 1, 'Timestep 0')]
        return self.patches + self.artists

    def animate_func(self, t):
        for k in range(len(self.tasks)):
            if self.tasks[k][1] >= int(t / self.delta_t):
                self.tasks[k][0].set_facecolor('orange')
            else:
                self.tasks[k][0].set_facecolor('white')
            self.tasks[k][0].set_edgecolor('none')
        for k in range(len(self.paths)):
            pos = self.get_state(t / self.delta_t, self.paths[k])
            self.agents[k].center = (pos[0], pos[1])
            self.agent_names[k].set_position((pos[0], pos[1] + 0.5))

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        ''''# check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at position {} at time {}".format(i, j, pos1, t))'''
        self.artists[-1].set_text('Timestep ' + str(int(t / self.delta_t)))
        return self.patches + self.artists

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            '''if t - int(t) < 0.6:
                delta = (t - int(t)) / 0.6
            else:
                delta = 1'''
            delta = t - int(t)
            # pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            pos = (pos_next - pos_last) * delta + pos_last
            return pos


file_name = "D:\lifelong\D_set\Set_4\instance_31_params"
my_map = FlowerMap(file_name + "\\parameter_31.txt")
paths = read_paths(file_name)
tasks = read_tasks(file_name)
animation = Animation(my_map, paths, tasks)
animation.show()
animation.save(file_name + "\\video.gif", 1000)