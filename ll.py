#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

array  = np.loadtxt("/home/jim/winglevlr/logs/AHRSD044.plog",
    usecols=(0,36))
fig, ax = plt.subplots()
ax.plot(array[:, 0], array[:, 1])
plt.show()


