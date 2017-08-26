import matplotlib.pyplot as plt
import pandas as pd
import os as os

df = pd.read_csv(os.path.expanduser('~') + "/.ros/gps_alignment_solution.csv")
plt.plot(df["gps_x"], df[" gps_y"], 'x', label="gps")
plt.plot(df[" world_x"], df[" world_y"], 'x', label="world")
plt.xlabel('x[m]')
plt.ylabel('y[m]')
plt.legend()
plt.show()