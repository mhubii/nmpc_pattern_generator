import numpy as np
import torch
import matplotlib.pyplot as plt

vel_gt = np.loadtxt('vel_gt.csv')
vel_pred = np.loadtxt('vel_pred.csv')

plt.subplot(131)
plt.plot(np.arange(vel_gt.shape[0]), vel_gt[:,0], label='GT x')
plt.plot(np.arange(vel_pred.shape[0]), vel_pred[:,0], label='Pred x')
plt.legend()
plt.subplot(132)
plt.plot(np.arange(vel_gt.shape[0]), vel_gt[:,1], label='GT y')
plt.plot(np.arange(vel_pred.shape[0]), vel_pred[:,1], label='Pred y')
plt.legend()
plt.subplot(133)
plt.plot(np.arange(vel_gt.shape[0]), vel_gt[:,2], label='GT z')
plt.plot(np.arange(vel_pred.shape[0]), vel_pred[:,2], label='Pred z')
plt.legend()
plt.show()
