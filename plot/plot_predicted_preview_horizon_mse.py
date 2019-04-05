import numpy as np
import matplotlib.pyplot as plt

# plot the mean squared net(x) - nmpc(x) over epochs 

# load data
data = np.genfromtxt('../build/bin/loss_hist.csv', delimiter=',')

# plot
plt.plot(data[1:,0], data[1:,1])
plt.show()

