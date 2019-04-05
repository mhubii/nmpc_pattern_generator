import numpy as np
import matplotlib.pyplot as plt

# plot the mean squared net(x) - nmpc(x) over epochs 

# load data
nmpc_data = np.genfromtxt('../build/bin/nmpc_com_preview_horizon.csv', delimiter=',')
net_data = np.genfromtxt('../build/bin/net_com_preview_horizon.csv', delimiter=',')

nmpc_jerks = np.genfromtxt('../build/bin/nmpc_jerks.csv', delimiter=',')
net_jerks = np.genfromtxt('../build/bin/net_jerks.csv', delimiter=',')

# plot
fig, ax = plt.subplots(nrows=1, ncols=2)

plt.subplot(121)
plt.plot(np.arange(0, net_jerks.shape[0]*0.01, 0.01), nmpc_jerks[:,0], label='nmpc x jerk')
plt.plot(np.arange(0, net_jerks.shape[0]*0.01, 0.01), net_jerks[:,0], linestyle='--', label='net x jerk')
plt.xlabel('time')
plt.ylabel('com x jerk')
plt.legend(loc='upper left')

plt.subplot(122)
plt.plot(np.arange(0, net_jerks.shape[0]*0.01, 0.01), nmpc_jerks[:,1], label='nmpc y jerk')
plt.plot(np.arange(0, net_jerks.shape[0]*0.01, 0.01), net_jerks[:,1], linestyle='--', label='net y jerk')
plt.xlabel('time')
plt.ylabel('com y jerk')
plt.legend(loc='upper left')

plt.tight_layout()

#plt.show()
plt.savefig('../img/learned_nmpc.png')
