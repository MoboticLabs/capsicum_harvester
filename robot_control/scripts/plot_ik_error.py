import numpy as np
import matplotlib.pyplot as plt

err_log = np.load('/tmp/ik_err_log.npy')

iters = np.arange(len(err_log))

plt.plot(iters, err_log[:, 0]*100, label='x error')
plt.plot(iters, err_log[:, 1]*100, label='y error')
plt.plot(iters, err_log[:, 2]*100, label='z error')

plt.xlabel('Iteration')
plt.ylabel('Position Error (m)')
plt.legend()
plt.grid(True)
plt.show()