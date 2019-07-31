import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command

plot_current = False
plot_preview = True

if (plot_current):

	# load the data
	data = np.genfromtxt('build/states.csv', delimiter=',', dtype=float)

	com_x = data[0, :]
	com_y = data[3, :]
	com_q = data[6, :]
	com_z = data[9, :]

	foot_x = data[10, :]
	foot_y = data[11,:]
	foot_q = data[12,:]

	# plot the data
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(com_x, com_y, com_z, label='com')
	ax.scatter(foot_x, foot_y, np.zeros([1, foot_x.size]), label='feet')

	plt.legend()
	plt.show()

if (plot_preview):

	# load the data on preview horizon
	preview = np.genfromtxt('build/states_preview.csv', delimiter=',', dtype=float)

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	for i in range(int(preview.shape[1]/16)):

		com_x_preview = preview[0, 16*i:16*(i+1)]
		com_y_preview = preview[1, 16*i:16*(i+1)]
		com_q_preview = preview[2, 16*i:16*(i+1)]
		com_z_preview = preview[3, 16*i:16*(i+1)]

		foot_x_preview = preview[4, 16*i:16*(i+1)]
		foot_y_preview = preview[5, 16*i:16*(i+1)]
		foot_q_preview = preview[6, 16*i:16*(i+1)]

		# plot the data
		ax.plot(com_x_preview, com_y_preview, com_z_preview, label=('com %d' % i))
		ax.scatter(foot_x_preview, foot_y_preview, np.zeros([1, foot_x_preview.size]), label='feet')

	ax.set_title('Preview Horizon')
	ax.set_xlabel('x/m')
	ax.set_ylabel('y/m')
	ax.set_zlabel('z/m')

        # spines color
        #ax.w_xaxis.line.set_color('white')
        #ax.w_yaxis.line.set_color('white')
        #ax.w_zaxis.line.set_color('white')

        # tick color
        #ax.xaxis._axinfo['tick']['color']='white'
        #ax.yaxis._axinfo['tick']['color']='white'
        #ax.zaxis._axinfo['tick']['color']='white'

        # tick number color
        #ax.tick_params(axis='x', colors='white')
        #ax.tick_params(axis='y', colors='white')
        #ax.tick_params(axis='z', colors='white')

        # label color
        #ax.xaxis.label.set_color('white')
        #ax.yaxis.label.set_color('white')
        #ax.zaxis.label.set_color('white')

	# plt.legend()
	# plt.show()
	plt.savefig('preview_horizon.png', dpi=900, transparent=True)
