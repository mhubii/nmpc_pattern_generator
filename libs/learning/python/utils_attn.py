from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
import numpy as np
from attn_model import AttnNet

dx = 2./120.
dy = 2./160.

x, y = np.mgrid[-1:1:dx, -1:1:dy]
pos = np.empty(x.shape + (2,))
pos[:, :, 0] = x; pos[:, :, 1] = y
rv = multivariate_normal([-1., 1.], [[0.1, 0.], [0., 0.1]])

k = rv.pdf(pos)


plt.imshow(k)
plt.show()
















"""

import torch
import cv2
from utils import crop
from attn_model import AttnNet

rgb = cv2.imread("/home/martin/Downloads/nmpc_pattern_generator/out/data/img/left_epoch_18_00719665.jpg", cv2.IMREAD_COLOR)
d = cv2.imread("/home/martin/Downloads/nmpc_pattern_generator/out/data/img/wls_disp_epoch_18_00719665.jpg", cv2.IMREAD_GRAYSCALE)

rgb = crop(rgb)
rgb = cv2.resize(rgb, (64, 64))

d = crop(d)
d = cv2.resize(d, (64, 64))

rgb = np.transpose(rgb, (2, 0, 1))
d = np.expand_dims(d, 0)
rgbd = np.concatenate([rgb, d], axis=0)

model = AttnNet(64, 2)

ten = torch.from_numpy(rgbd).float()
ten = ten.unsqueeze(0)

g, c1, c2, c3 = model(ten)
print(c1.shape)

ones = torch.ones([1, 4, 64, 64])
"""