import torch
import torch.nn as nn
import numpy as np
import torchvision
from torch.autograd import Variable
import cv2
import tqdm

from unet_model import UNet
import utils

model = UNet(utils.RGBD_INPUT_SHAPE, 2, 1).cuda()
model.load_state_dict(torch.load('trained_rgbd.pt'))
model.eval()

# input to optimize for
max_in = torch.rand(1, utils.IMAGE_CHANNELS, utils.RESIZED_IMAGE_HEIGHT, utils.RESIZED_IMAGE_WIDTH).cuda()
max_in = max_in.mul(2.).sub(1.)
max_in = Variable(max_in, requires_grad=True)

# desired velocity
vel_des = np.array([0., -0.2])
vel_des = torch.from_numpy(vel_des).float().cuda()
vel_max = np.array([0.15, 0.2])
vel_max = torch.from_numpy(vel_max).float().cuda()

# load images
data_dir = "/home/martin/Downloads/nmpc_pattern_generator/out/"
img_path, velocities = utils.load_data(data_dir)

# optimizer
opt = torch.optim.Adam([max_in], lr=0.1)

# criterion
criterion = nn.MSELoss().cuda()

# optimizing loop
for i in tqdm.tqdm(range(100)):
    # load images
    rgb, d = utils.load_rgbd(data_dir, img_path[i][0], img_path[i][1])

    # Crop.
    rgb = utils.crop(rgb)
    d = utils.crop(d)

    # Resize.
    rgb = utils.resize(rgb)
    d = utils.resize(d)

    # Normalize.
    rgb = utils.normalize(rgb)
    d = utils.normalize(d)

    # Change HxWxC to CxHxW.
    rgb = np.transpose(rgb, (2, 0, 1))
    d = np.expand_dims(d, 2)
    d = np.transpose(d, (2, 0, 1))

    # Concatenate rgb, d to rgbd image.  
    rgbd = np.concatenate([rgb, d], axis=0)
    rgbd = torch.from_numpy(rgbd).float()
    rgbd = torch.unsqueeze(rgbd, 0).cuda()

    opt.zero_grad()
    vel = model(max_in)
    loss_vel = criterion(vel, vel_des)
    loss_img = criterion(max_in, rgbd)
    loss = loss_img + loss_vel
    loss.backward()
    opt.step()

max_in = max_in.detach().cpu().squeeze()
max_in = max_in.permute([1, 2, 0]).numpy()
max_in = cv2.resize(max_in, (320, 240))

# normalize
rgb = cv2.cvtColor(max_in[:,:,:3], cv2.COLOR_BGR2RGB)
d = np.array(np.expand_dims(max_in[:,:,3], axis=2))

cv2.normalize(rgb,  rgb, 0, 255, cv2.NORM_MINMAX)
cv2.normalize(d,  d, 0, 255, cv2.NORM_MINMAX)

cv2.imwrite('opt_rgb.png', rgb)
cv2.imwrite('opt_depth.png', d)
