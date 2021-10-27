#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

c = {"time":0, "roll":36, "pitch":39}


array  = np.loadtxt("./logs/AHRSD044.plog",
    usecols=(c["time"],c["roll"],c["pitch"]))
fig, ax = plt.subplots()
ax.plot(array[:, 0], array[:, 1], lw=.5)
ax.plot(array[:, 0], array[:, 2], lw=.5)

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()
plt.tight_layout()
plt.subplots_adjust(left=0.03, right=0.97, top=0.97, bottom=0.03)
plt.show(block=True)

