import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import pandas as pd
import os as os

def newline(p1, p2):
    ax = plt.gca()
    l = mlines.Line2D([p1[0],p2[0]], [p1[1],p2[1]], color='grey')
    ax.add_line(l)
    return l

df = pd.read_csv(os.path.expanduser('~') + "/.ros/gps_alignment_solution.csv")

for i in range(0,  df["world_x"].size, 100):
    p1 = [df["gps_x"][i],df["gps_y"][i]]
    p2 = [df["world_x"][i], df["world_y"][i]]
    newline(p1,p2)
    
plt.plot(df["gps_x"], df["gps_y"], 'x', label="gps")
plt.plot(df["world_x"], df["world_y"], 'x', label="world")
plt.xlabel('x[m]')
plt.ylabel('y[m]')




plt.legend()
plt.show()
