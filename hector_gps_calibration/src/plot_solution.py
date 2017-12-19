import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy as np
import pandas as pd
import os as os
colors = plt.cm.Set1(np.linspace(0, 1, 9))
from matplotlib2tikz import save as tikz_save
def newline(p1, p2):
    ax = plt.gca()
    l = mlines.Line2D([p1[0],p2[0]], [p1[1],p2[1]], color='grey')
    ax.add_line(l)
    return l

name = "gps_alignment_solution_final"
df = pd.read_csv(os.path.expanduser('~') + "/.ros/" + name + ".csv")

for i in range(0,  df["world_x"].size, 3):
    p1 = [df["gps_x"][i],df["gps_y"][i]]
    p2 = [df["world_x"][i], df["world_y"][i]]
    newline(p1,p2)
    
plt.scatter(df["world_x"], df["world_y"], label="world", c='orange', marker="+")
#plt.scatter(df["gps_x"], df["gps_y"], label="gps", c=df["covariance"], marker="+")
plt.scatter(df["gps_x"], df["gps_y"], label="gps", c=df["covariance"], marker="+")
plt.xlabel('East [m]')
plt.ylabel('North [m]')
plt.xlim((475245, 475390))
plt.ylim((5524880, 5525075))
plt.legend()
tikz_save(name + '.tikz')

plt.show()
