# Licensed under MIT licence, see LICENSE for details.
# Copyright Ote Robotics Ltd. 2020

import pandas as pd
import sys
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_json(sys.argv[1], orient="records", lines=True)
df.set_index("server_epoch_ms")
df[["server_epoch_ms", "x"]].plot(x="server_epoch_ms")
plt.title("x position")
df[["server_epoch_ms", "xvel_raw", "xvel_hd5"]].plot(x="server_epoch_ms")
plt.title("x velocity (from numerical differentiation)")

dt = np.diff(np.array(df[["server_epoch_ms"]]).T*1e-3)
dt = dt[dt < 1]
plt.figure()
plt.hist(dt, bins=100)
plt.title("histogram of camera frame dt")

plt.show()
