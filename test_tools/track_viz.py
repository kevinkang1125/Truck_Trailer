import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si
import os
control_1 = np.random.randint([100,-200],[200,-50])
control_2 = np.random.randint([-200,-300],[200,-150])
print(control_1,control_2)
#points = np.array([[0, 0], [160, -150], [120, -240], [0, -350]])
points = np.array([[0, 0], control_1, control_2, [0, -350]])
direction = np.random.choice([-1,1])

x = points[:,0]
y = points[:,1]
y = y*direction
t = range(len(x))

ipl_t = np.linspace(0.0, len(points) - 1, 100)

x_tup = si.splrep(t, x, k=3)
y_tup = si.splrep(t, y, k=3)
x_i = si.splev(ipl_t, x_tup)
y_i = si.splev(ipl_t, y_tup) 

fig = plt.figure()
#ax = fig.add_subplot(111)
plt.plot(y, x, label='original')
plt.plot(y_i, x_i, label='spline')
plt.plot(0,0,'r^',label='start point')
#plt.xlim([-250, 250])
#plt.ylim([-350, 350])
plt.legend()
plt.show()