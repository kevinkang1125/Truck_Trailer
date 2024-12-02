import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

reward = pd.read_csv("monitor.csv")
print(reward)
plt.plot(reward[" r "])
plt.show()