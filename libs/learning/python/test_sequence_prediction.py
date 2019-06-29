import torch
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

from unet_model import UNet
from unet_model import GDUNet
from model import RGBDCNN
import utils
from utils import load_rgbd

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command

#model = RGBDCNN(utils.RGBD_INPUT_SHAPE, 3, 1)
#model.load_state_dict(torch.load('trained_rgbd.pt'))
model = UNet(utils.RGBD_INPUT_SHAPE, 2, 1).cuda()
model.load_state_dict(torch.load('trained_rgbd.pt'))
#model = GDUNet(utils.GD_INPUT_SHAPE, 3, 1)
#model.load_state_dict(torch.load('trained_gd.pt'))
model.eval()

data_dir = "/home/martin/Downloads/nmpc_pattern_generator/out/"
img_paths, velocities = utils.load_unshuffeled_data(data_dir)

vel_gt = []
vel_pred = []

N = 25000

# Maximal velocities.
#vel_max = np.array([0.15, 0.02, 0.2])
vel_max = np.array([0.15, 0.2])
vel_max = torch.from_numpy(vel_max).float().cuda()
"""
for i in range(N,N+1000,1):
    rgb, d = load_rgbd(data_dir, img_path[i][0], img_path[i][1])

    # Crop.
    rgb = utils.crop(rgb)
    d = utils.crop(d)

    # Resize.
    rgb = utils.resize(rgb)
    d = utils.resize(d)

    # Filter the red channel.
    #rgb = utils.filter(rgb)

    # Normalize.
    rgb = utils.normalize(rgb)
    d = utils.normalize(d)

    # Change HxWxC to CxHxW.
    rgb = np.transpose(rgb, (2, 0, 1))
    d = np.expand_dims(d, 2)
    d = np.transpose(d, (2, 0, 1))

    # Concatenate rgb, d to rgbd image.  
    #rgb = np.expand_dims(rgb, 0)
    #d = np.expand_dims(d, 0)  
    rgbd = np.concatenate([rgb, d], axis=0)

    rgbd = torch.from_numpy(rgbd).float()
    rgbd = torch.unsqueeze(rgbd, 0).cuda()
"""
sequence_length = 5

for i in range(N,N+1000,1):
    sample = {'img_left': [], 'img_wls_disp': [], 'vel': []} 

    for j in range(-sequence_length+1,1): # for the correct ordering in temporal dimension
        img_left, img_wls_disp = load_rgbd(data_dir, img_paths[i+j][0], img_paths[i+j][1])

        vel = velocities[i+j]

        sample['img_left'].append(img_left)
        sample['img_wls_disp'].append(img_wls_disp)

    for j in range(sequence_length):

        sample['img_left'][j] = utils.crop(sample['img_left'][j])
        sample['img_wls_disp'][j] = utils.crop(sample['img_wls_disp'][j])

        # Resize.
        sample['img_left'][j] = utils.resize(sample['img_left'][j])
        sample['img_wls_disp'][j]	 = utils.resize(sample['img_wls_disp'][j])

        sample['img_left'][j] = utils.normalize(sample['img_left'][j])
        sample['img_wls_disp'][j] = utils.normalize(sample['img_wls_disp'][j])

        # Change HxWxC to CxHxW.
        sample['img_left'][j] = np.transpose(sample['img_left'][j], (2, 0, 1))
        #img_wls_dips = np.transpose(img_wls_disp, (2, 0, 1)) TODO

        # Concatenate rgb, d to rgbd image.    
        sample['img_wls_disp'][j]  = np.expand_dims(sample['img_wls_disp'][j], 0)
        sample['img_left'][j] = np.concatenate([sample['img_left'][j], sample['img_wls_disp'][j]], axis=0)

    imgs_rgbd = np.array(sample['img_left'])
    imgs_rgbd = torch.from_numpy(imgs_rgbd).float().cuda()
    imgs_rgbd = imgs_rgbd.unsqueeze(0)

    #vel = model(rgbd)
    vel = model(imgs_rgbd)
    vel = torch.mul(vel, vel_max)

    vel_gt.append(velocities[i])
    vel_pred.append(vel.squeeze().detach().cpu().numpy())


vel_gt = np.array(vel_gt)
vel_pred = np.array(vel_pred)

np.savetxt('vel_gt.csv', vel_gt)
np.savetxt('vel_pred.csv', vel_pred)

# average and see what happens
#vel_pred_avg = []
#n = 1
#for i in range(int(vel_pred.shape[0]/n)):
#    vel_pred_avg.append(vel_pred[i*n:i*(n+1)].mean(axis=0))

#vel_pred_avg = np.array(vel_pred_avg)

plt.subplot(121)
plt.plot(np.arange(vel_pred.shape[0]), vel_pred[:,0], label=r'Predicted Commands $v_x$')
plt.plot(np.arange(vel_gt.shape[0]), vel_gt[:,0], label=r'User Commands $v_x$')
plt.xlabel(r'Frames[\#]')
plt.ylabel(r'$v_x[\frac{\text{m}}{\text{s}}]$')
plt.legend()
plt.subplot(122)
plt.plot(np.arange(vel_pred.shape[0]), vel_pred[:,1], label=r'Predicted Commands $\omega_z$')
plt.plot(np.arange(vel_gt.shape[0]), vel_gt[:,1], label=r'User Commands $\omega_z$')
plt.xlabel(r'Frames[\#]')
plt.ylabel(r'$\omega_z[\frac{\text{rad}}{\text{s}}]$')
plt.legend()
plt.tight_layout()
plt.show()
#plt.savefig('predicted_behaviour.pdf')

