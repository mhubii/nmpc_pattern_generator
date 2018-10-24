import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_csv(loc):
    data = np.genfromtxt(loc, delimiter=',', dtype=float)
    return data


def plot_trajectories(data, loc):
    a = 0
    z = data.shape[0]
    com_x = data[a:z, 0]
    com_y = data[a:z, 3]
    com_z = data[a:z, 6]
    left_x = data[a:z, 13]
    left_y = data[a:z, 14]
    left_z = data[a:z, 15]
    right_x = data[a:z, 18]
    right_y = data[a:z, 19]
    right_z = data[a:z, 20]


    plt.plot(com_x, com_y, label='com')
    plt.plot(left_x, left_y, label='left foot')
    plt.plot(right_x, right_y, label='right foot')
    plt.xlabel('x/m')
    plt.ylabel('y/m')
    plt.title('Walking Trajectories')
    plt.legend()
    plt.savefig(loc)


def plot_3d(data, ref_data):
    a = 0
    z = data.shape[0]
    com_x = data[a:z, 0]
    com_y = data[a:z, 3]
    com_z = data[a:z, 6]
    com_x_ref = ref_data[a:z, 0]
    com_y_ref = ref_data[a:z, 3]
    com_z_ref = ref_data[a:z, 6]
    left_x = data[a:z, 13]
    left_y = data[a:z, 14]
    left_z = data[a:z, 15]
    right_x = data[a:z, 17]
    right_y = data[a:z, 18]
    right_z = data[a:z, 19]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(com_x, com_y, com_z, label='com')
    ax.plot(com_x_ref, com_y_ref, com_z_ref, label='com ref')
    #plt.plot(com_x, com_y, label='com')
    #plt.plot(com_x_ref, com_y_ref, c='orange', label='com ref')
    ax.plot(left_x, left_y, left_z, label='left foot')
    ax.plot(right_x, right_y, right_z, label='right foot')

    plt.legend()
    plt.title('Walking Trajectories')
    plt.show()

def plot_z(data):
    a = 0
    z = data.shape[0]

    right_x = data[a:z, 18]
    right_y = data[a:z, 19]
    right_z = data[a:z, 20]

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



if __name__ == '__main__':
    #loc_in = '/home/martin/Documents/heicub_walking/build/example_nmpc_generator_interpolated_results.csv'
    loc_in = '/home/martin/Documents/heicub_walking/build/test.csv'
    loc_in_ref = '/home/martin/Documents/heicub_walking/build/test_com_feedback.csv'
    loc_out = '/home/martin/Documents/heicub_walking/build/generated_nmpc_pattern.png'
    data = load_csv(loc_in)
    ref_data = load_csv(loc_in_ref)
    # plot_trajectories(data, loc_out)
    plot_3d(data, ref_data)
    #plot_dif(ref_data)
    #plot_z(data)
    #plot_o(data)
