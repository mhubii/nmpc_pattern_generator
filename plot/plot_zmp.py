import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command


def load_csv(loc):
    data = np.genfromtxt(loc, delimiter=',', dtype=float)
    return data


def plot_zmp(data, loc):
    a = 0
    z = data.shape[0]
    com_x = data[a:z, 0]
    com_y = data[a:z, 3]
    com_z = data[a:z, 6]
    zmp_x = data[a:z, 10]
    zmp_y = data[a:z, 11]
    zmp_z = data[a:z, 12]

    #plt.plot(com_x, com_y, label='com')
    plt.plot(zmp_x, zmp_y, label='zmp')
    plt.xlabel('x/m')
    plt.ylabel('y/m')
    plt.title('Zero Moment Point')
    plt.legend()
    #plt.savefig(loc)
    #plt.show()


def ft_to_zmp(ft_data, pos_data):
    # distance sole ankle (force torque sensor)
    d = 0.02986

    force_left   = ft_data[:, 0:3]
    torque_left    = ft_data[:, 3:6]
    force_right  = ft_data[:, 6:9]
    torque_right   = ft_data[:, 9:12]

    # zmp in feet frames
    p_lx = (-torque_left[:,1] - force_left[:,0]*d)/force_left[:,2]
    p_ly = ( torque_left[:,0] - force_left[:,1]*d)/force_left[:,2]
    p_rx = (-torque_right[:,1] - force_right[:,0]*d)/force_right[:,2]
    p_ry = ( torque_right[:,0] - force_right[:,1]*d)/force_right[:,2] 

    # transform to world frame
    lfx = pos_data[:, 13]
    lfy = pos_data[:, 14]
    rfx = pos_data[:, 17]
    rfy = pos_data[:, 18]

    p_lx += lfx
    p_ly += lfy
    p_rx += rfx
    p_ry += rfy    

    # zmp in world frame
    p_x =  (p_rx*force_right[:,2] + p_lx*force_left[:,2]) / (force_right[:,2] + force_left[:,2])
    p_y =  (p_ry*force_right[:,2] + p_ly*force_left[:,2]) / (force_right[:,2] + force_left[:,2])

    return p_x, p_y


if __name__ == '__main__':
    #loc_in = '../build/bin/example_nmpc_generator_interpolated_results.csv'
    loc_in = '../out/data/trajectories_epoch_9.csv'
    loc_ft = '../out/data/force_torque_epoch_9.csv'
    loc_out = '../img/generated_nmpc_pattern.png'
    data = load_csv(loc_in)
    plot_zmp(data, loc_out)
    
    data_ft = load_csv(loc_ft)
    zmp_x, zmp_y = ft_to_zmp(data_ft, data)
    plt.plot(zmp_x, zmp_y, label='measured zmp')
    plt.legend()
    plt.show()

