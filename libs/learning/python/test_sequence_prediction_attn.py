import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import cv2
import torchvision.utils as ut

from attn_model import AttnNet
import utils
from utils import load_rgbd

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}'] #for \text command

def visualize_attn_softmax(I, c, up_factor, nrow):
    # image
    img = I.permute((1,2,0)).cpu().numpy()
    # compute the heatmap
    N,C,W,H = c.size()
    a = F.softmax(c.view(N,C,-1), dim=2).view(N,C,W,H)
    if up_factor > 1:
        a = F.interpolate(a, scale_factor=up_factor, mode='bilinear', align_corners=False)
    attn = ut.make_grid(a, nrow=nrow, normalize=True, scale_each=True)
    attn = attn.permute((1,2,0)).mul(255).byte().cpu().numpy()
    attn = cv2.applyColorMap(attn, cv2.COLORMAP_JET)
    attn = cv2.cvtColor(attn, cv2.COLOR_BGR2RGB)
    attn = np.float32(attn) / 255
    # add the heatmap to the image
    vis = 0.6 * img + 0.4 * attn
    vis = cv2.resize(vis, (256, 256))
    return vis

#model = RGBDCNN(utils.RGBD_INPUT_SHAPE, 3, 1)
#model.load_state_dict(torch.load('trained_rgbd.pt'))
model = AttnNet(utils.RESIZED_IMAGE_HEIGHT, 2).cuda()
model.load_state_dict(torch.load('trained_attn.pt'))
#model = GDUNet(utils.GD_INPUT_SHAPE, 3, 1)
#model.load_state_dict(torch.load('trained_gd.pt'))
model.eval()

data_dir = "/home/martin/Downloads/nmpc_pattern_generator/out/"
img_paths, velocities = utils.load_unshuffeled_data(data_dir)

vel_gt = []
vel_pred = []

N = 17000

# Maximal velocities.
#vel_max = np.array([0.15, 0.02, 0.2])
vel_max = np.array([0.15, 0.2])
vel_max = torch.from_numpy(vel_max).float().cuda()

sequence_length = 5

for i in range(N,N+300,1):
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

    vel, c1, c2, c3 = model(imgs_rgbd)
    vel = torch.mul(vel, vel_max)

    vel_gt.append(velocities[i])
    vel_pred.append(vel.squeeze()[sequence_length-1].detach().cpu().numpy())

    # print(imgs_rgbd[0, 0, :3].shape)
    # print(c3.shape)
    # vis = visualize_attn_softmax(imgs_rgbd[0, 0, :3], c3[0].unsqueeze(0), 1, 64)
    # cv2.imshow("img", vis)
    # cv2.waitKey(0)

vel_gt = np.array(vel_gt)
vel_pred = np.array(vel_pred)

np.savetxt('vel_gt.csv', vel_gt)
np.savetxt('vel_pred.csv', vel_pred)

# average and see what happens
# vel_pred_avg = []
# n = 1
# for i in range(int(vel_pred.shape[0]/n)):
#    vel_pred_avg.append(vel_pred[i*n:i*(n+1)].mean(axis=0))

# vel_pred_avg = np.array(vel_pred_avg)

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

