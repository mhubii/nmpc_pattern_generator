import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command


def load_csv(loc):
    data = np.genfromtxt(loc, delimiter=',', dtype=float)
    return data


def plot_trajectories(data, loc):
    a = 0
    z = data.shape[0]
    stride = 1
    com_x = data[a:z:stride, 0]
    com_y = data[a:z:stride, 3]
    com_z = data[a:z:stride, 6]
    left_x = data[a:z:stride, 13]
    left_y = data[a:z:stride, 14]
    left_z = data[a:z:stride, 15]
    right_x = data[a:z:stride, 17]
    right_y = data[a:z:stride, 18]
    right_z = data[a:z:stride, 19]


    plt.plot(com_x, com_y, label='com')
    plt.plot(left_x, left_y, label='left foot')
    plt.plot(right_x, right_y, label='right foot')
    plt.xlabel('x/m')
    plt.ylabel('y/m')
    plt.title('Walking Trajectories')
    plt.legend()
    plt.show()
    #plt.savefig(loc)


def plot_3d(data, loc):
    a = 0
    z = data.shape[0]
    stride = 1
    com_x = data[a:z:stride, 0]
    com_y = data[a:z:stride, 3]
    com_z = data[a:z:stride, 6]
    left_x = data[a:z:stride, 13]
    left_y = data[a:z:stride, 14]
    left_z = data[a:z:stride, 15]
    right_x = data[a:z:stride, 17]
    right_y = data[a:z:stride, 18]
    right_z = data[a:z:stride, 19]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(com_x, com_y, com_z, label='CoM')
    ax.plot(left_x, left_y, left_z, label='Left Foot')
    ax.plot(right_x, right_y, right_z, label='Right Foot')

    ax.set_xlabel(r'$\text{x}\,/\,\text{m}$')
    ax.set_ylabel(r'$\text{y}\,/\,\text{m}$')
    ax.set_zlabel(r'$\text{z}\,/\,\text{m}$')	

    ax.view_init(20, 60) 

    plt.legend()
    plt.title('Interpolated Walking Trajectories')
    plt.show()
    #plt.savefig(loc, dpi=900, transparent=True)
    #plt.savefig(loc)


def plot_3d_with_feedback(data, data_feedback, loc):
    a = 0
    z = data.shape[0]
    stride = 1
    com_x = data[a:z:stride, 0]
    com_y = data[a:z:stride, 3]
    com_z = data[a:z:stride, 6]
    left_x = data[a:z:stride, 13]
    left_y = data[a:z:stride, 14]
    left_z = data[a:z:stride, 15]
    right_x = data[a:z:stride, 17]
    right_y = data[a:z:stride, 18]
    right_z = data[a:z:stride, 19]

    com_x_feedback = data_feedback[:,0]
    com_y_feedback = data_feedback[:,1]
    com_z_feedback = data_feedback[:,2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(com_x, com_y, com_z, label='CoM')
    ax.plot(left_x, left_y, left_z, label='Left Foot')
    ax.plot(right_x, right_y, right_z, label='Right Foot')
    ax.plot(com_x_feedback, com_y_feedback, com_z_feedback, label='CoM Feedback')

    ax.set_xlabel(r'$\text{x}\,/\,\text{m}$')
    ax.set_ylabel(r'$\text{y}\,/\,\text{m}$')
    ax.set_zlabel(r'$\text{z}\,/\,\text{m}$')	

    ax.view_init(20, 60) 

    plt.legend()
    plt.title('Interpolated Walking Trajectories')
    plt.show()
    #plt.savefig(loc, dpi=900, transparent=True)
    #plt.savefig(loc)


def plot_raw_3d(data, loc):
    a = 0
    z = data.shape[0]

    com_x = data[a:z, 0]
    com_y = data[a:z, 3]
    com_q = data[a:z, 6]
    com_z = data[a:z, 9]

    left_x = data[a:z, 10]
    left_y = data[a:z, 11]
    right_x = data[a:z, 13]
    right_y = data[a:z, 14]

    # plot the data
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(com_x, com_y, com_z, label='CoM', marker='o', alpha=1.)
    ax.scatter(left_x, left_y, np.zeros([1, left_x.size]), label='Left Foot', marker='o', alpha=1.)
    ax.scatter(right_x, right_y, np.zeros([1, right_x.size]), label='Right Foot', marker='o', alpha=1.)

    ax.set_xlabel(r'$\text{x}\,/\,\text{m}$')
    ax.set_ylabel(r'$\text{y}\,/\,\text{m}$')
    ax.set_zlabel(r'$\text{z}\,/\,\text{m}$')	

    ax.view_init(20, 60) 

    plt.legend()
    plt.title('Raw Walking Trajectories')
    plt.savefig(loc, dpi=900, transparent=True)
    #plt.show()

def plot_z(data):
    a = 0
    z = data.shape[0]

    right_x = data[a:z, 17]
    right_y = data[a:z, 18]
    right_z = data[a:z, 19]

    time = np.linspace(0, right_x.shape[0]*0.01, right_x.shape[0]) 

    plt.plot(time, right_x, label='right x')
    plt.plot(time, right_y, label='right y')
    plt.plot(time, right_z, label='right z')
    plt.grid()
    plt.legend()
    plt.show()

def plot_o(data):
    a = 0
    z = data.shape[0]

    left_o = data[a:z, 17]
    right_o = data[a:z, 22]

    time = np.linspace(0, right_o.shape[0]*0.01, right_o.shape[0]) 

    plt.plot(time, left_o, label='left omega')
    plt.plot(time, right_o, label='right omega')
    plt.grid()
    plt.legend()
    plt.show()

def plot_dif(data):
    a = 0
    z = data.shape[0]

    dx = data[a:z, 0]
    dy = data[a:z, 1]
    dz = data[a:z, 2]

    time = np.linspace(0, dx.shape[0]*0.01, dx.shape[0]) 

    plt.plot(time, dx, label='dx')
    plt.plot(time, dy, label='dy')
    plt.plot(time, dz, label='dz')

    plt.legend()
    plt.show()

def plot_with_obstacle(loc):
    data = load_csv(loc)

    a = 0
    z = 1000#data.shape[0]
    com_x = data[a:z, 0]
    com_y = data[a:z, 3]
    com_z = data[a:z, 6]
    left_x = data[a:z, 13]
    left_y = data[a:z, 14]
    left_z = data[a:z, 15]
    right_x = data[a:z, 17]
    right_y = data[a:z, 18]
    right_z = data[a:z, 19]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(com_x, com_y, label='CoM')
    ax.scatter(left_x, left_y, label='Left Foot')
    ax.scatter(right_x, right_y, label='Right Foot')

    ax.set_xlabel(r'$\text{x}\,/\,\text{m}$')
    ax.set_ylabel(r'$\text{y}\,/\,\text{m}$')

    # circle with radius = radius + margin + max foot
    circle = plt.Circle(([10, 10]), 0.5, linestyle='-', color='gray', fill=False, label='Obstacle')
    ax.add_patch(circle)

    circle = plt.Circle(([10, 10]), 0.5 + 0.2 + 0.2, linestyle='--', color='gray', fill=False, label='Security Margin')
    ax.add_patch(circle)

    #ax.set_xlim( 0.0, 4.5)
    #ax.set_ylim(-1.0, 2.5)

    plt.legend()
    plt.title('Walking Trajectories')
    plt.show()



if __name__ == '__main__':
    #loc_in = '/home/martin/Downloads/nmpc_pattern_generator/out/behavioural_augmentation_measurements/trajectories.csv'
    #loc_in = '../build/bin/example_nmpc_generator_interpolated_results.csv'
    loc_in = '../build/bin/user_controlled_walking_trajectories.csv'
    #loc_in = '../out/interpolated_states.csv'
    #loc_in = '../build/bin/test_com_feedback.csv'
    loc_out = '../img/interpolated_results.png'
    data = load_csv(loc_in)
    #plot_trajectories(data, loc_out)
    #plot_3d(data, loc_out)

    #loc_in = '../out/raw_states.csv'
    #loc_out = '../img/raw_results.png'
    #data = load_csv(loc_in)
    #plot_raw_3d(data, loc_out)
    #plot_with_obstacle(loc_in)
    #plot_z(data)
    #plot_o(data)

    # CoM feedback.
    loc_com = '../build/bin/com.csv'
    data_feedback = load_csv(loc_com)
    plot_3d_with_feedback(data, data_feedback, loc_out)
