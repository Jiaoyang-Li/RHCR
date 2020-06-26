#!/usr/bin/env python3
import pandas as pd
from re import split
import numpy as np
import scipy.cluster.hierarchy as shc
import scipy.spatial.distance as ssd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib
matplotlib.use('TkAgg')

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.cluster.hierarchy.linkage.html#scipy.cluster.hierarchy.linkage
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.cluster.hierarchy.fcluster.html

def read_map(fname):
    with open(fname, "r") as f:
        f.readline()  # skip "Grid size (rows, cols)"
        line = f.readline()
        line = split(",", line[:-1])
        ylim = int(line[0])
        xlim = int(line[1])
        f.readline()  # skip the headers
        types = []
        stations = []
        coordinates = []
        for line in f.readlines():
            data = split(",", line[:-1])
            types.append(data[1])
            stations.append(data[2])
            coordinates.append([int(data[4]), int(data[3])])
        return xlim, ylim, types, stations, coordinates


map_name = "../maps/sorting_e=inf.grid"

dist_matrix = pd.read_csv('distances.csv',  header=None)
N = len(dist_matrix)
condensed = ssd.squareform(dist_matrix)
from datetime import datetime
start = datetime.now()
cluster = shc.linkage(condensed, method='complete')
print(datetime.now()-start)
labels = shc.fcluster(cluster, 10, 'maxclust')  # 'maxclust')distance
M = max(labels)
cluster_size = [0 for i in range(M + 1)]
for label in labels:
    cluster_size[label] += 1
print(M)
print(cluster_size[1:])

plt.subplot(2, 1, 2)
cluster_size.sort(reverse=True)
plt.title("Class sizes {}".format(cluster_size[:min(10, M)]), loc='center')
dend = shc.dendrogram(cluster, labels=range(N))
# plt.axhline(y=10, color='r', linestyle='--')

ax = plt.subplot(2, 2, 1)
xlim, ylim, types, stations, coordinates = read_map(map_name)
my_map = [[1 for _ in range(ylim)] for _ in range(xlim)]
for i in range(len(types)):
    if types[i] == 'Obstacle':
        my_map[coordinates[i][0]][coordinates[i][1]] = 0
plt.imshow(my_map, cmap=cm.Greys_r)
agents = pd.read_csv('agents.csv')
xs = [coordinates[i][1] for i in agents['start']]
ys = [coordinates[i][0] for i in agents['start']]
scatter = plt.scatter(xs, ys, c=labels, cmap="Spectral")
# produce a legend with the unique colors from the scatter
# legend = plt.legend(*scatter.legend_elements(), loc='center right', bbox_to_anchor=(1.3, 0.5), title="Classes")
# ax.add_artist(legend)
# plt.legend()
plt.title("Start locations")

plt.subplot(2, 2, 2)
plt.imshow(my_map, cmap=cm.Greys_r)
xs = [coordinates[i][1] for i in agents['goal']]
ys = [coordinates[i][0] for i in agents['goal']]
plt.scatter(xs, ys, c=labels, cmap="Spectral")
plt.title("First goal locations")

plt.show()