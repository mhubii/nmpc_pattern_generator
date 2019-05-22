import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command

history = np.loadtxt("history.csv")
epochs = 20

plt.plot(np.linspace(1, epochs + 1, len(history)), history, label='loss')
plt.title("Behavioural Cloning Progress")
plt.ylabel(r"$\text{MSE}\,/\,(\frac{\text{m}^2}{\text{s}^2})$")
plt.xlabel(r"$\text{Epochs}\,/\,\#$")
plt.legend()
plt.grid()
plt.savefig("behavioural_cloning_progress.pdf")#, dpi=900)
#plt.show()

