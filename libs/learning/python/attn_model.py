import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from scipy.stats import multivariate_normal
import utils

"""
class AttnNet(nn.Module):
    def __init__(self, h, w, dof):
        super(AttnNet, self).__init__()
        self.h = h
        self.w = w
        self.dx = 2./h
        self.dy = 2./w

        # attention layers
        self.att_conv1 = nn.Conv2d(4, 16, 5, 2)
        self.att_conv2 = nn.Conv2d(16, 16, 5, 2)
        self.att_conv3 = nn.Conv2d(16, 16, 5, 2)
        self.att_fc = nn.Linear(self.get_att_numel(h, w), 2)

        # attention based decission layers
        self.conv1 = nn.Conv2d(4, 16, 5, 2)
        self.conv2 = nn.Conv2d(16, 16, 5, 2)
        self.conv3 = nn.Conv2d(16, 16, 5, 2)
        self.fc1 = nn.Linear(self.get_numel(h, w), 2)
        self.fc2 = nn.Linear(4, dof)

    def forward(self, x):
        # determine mu
        mu = F.relu(self.att_conv1(x))
        mu = F.relu(self.att_conv2(mu))
        mu = F.relu(self.att_conv3(mu))
        mu = mu.view(mu.shape[0], -1)
        mu = torch.tanh(self.att_fc(mu))

        # determine decission
        k = self.gaussian(mu.detach().numpy()).float()
        x = x.mul(k)
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = x.view(x.shape[0], -1)
        x = F.relu(self.fc1(x))

        # concatenate
        x = torch.cat((x, mu), dim=1)
        x = self.fc2(x)

        return torch.tanh(x), k

    def gaussian(self, mu):
        x, y = np.mgrid[-1:1:self.dx, -1:1:self.dy]
        pos = np.empty(x.shape + (2,))
        pos[:, :, 0] = x; pos[:, :, 1] = y

        out = torch.zeros([mu.shape[0], 4, self.h, self.w])

        for i in range(mu.shape[0]):
            rv = multivariate_normal(mu[0], [[0.1, 0.], [0., 0.1]])

            # return kernel
            k = np.expand_dims(rv.pdf(pos), 0)
            k = np.tile(k, (4, 1, 1))
            out[i] =  torch.from_numpy(k)
        
        return out

    def get_numel(self, h, w):
        inp = torch.ones([1, 4, h, w])
        inp = self.conv1(inp)
        inp = self.conv2(inp)
        inp = self.conv3(inp)

        return inp.numel()

    def get_att_numel(self, h, w):
        inp = torch.ones([1, 4, h, w])
        inp = self.att_conv1(inp)
        inp = self.att_conv2(inp)
        inp = self.att_conv3(inp)

        return inp.numel()
"""




# as implemented by https://github.com/SaoYan/LearnToPayAttention/blob/master/blocks.py
class ConvBlock(nn.Module):
    def __init__(self, in_features, out_features, num_conv, pool=False):
        super(ConvBlock, self).__init__()
        features = [in_features] + [out_features for i in range(num_conv)]
        layers = []
        for i in range(len(features)-1):
            layers.append(nn.Conv2d(in_channels=features[i], out_channels=features[i+1], kernel_size=3, padding=1, bias=True))
            layers.append(nn.BatchNorm2d(num_features=features[i+1], affine=True, track_running_stats=True))
            layers.append(nn.ReLU())
            if pool:
                layers.append(nn.MaxPool2d(kernel_size=2, stride=2, padding=0))
        self.op = nn.Sequential(*layers)
    def forward(self, x):
        return self.op(x)

class LinearAttentionBlock(nn.Module):
    def __init__(self, in_features, normalize_attn=True):
        super(LinearAttentionBlock, self).__init__()
        self.normalize_attn = normalize_attn
        self.op = nn.Conv2d(in_channels=in_features, out_channels=1, kernel_size=1, padding=0, bias=False)
    def forward(self, l, g):
        N, C, W, H = l.size()
        c = self.op(l+g) # batch_sizex1xWxH
        if self.normalize_attn:
            a = F.softmax(c.view(N,1,-1), dim=2).view(N,1,W,H)
        else:
            a = torch.sigmoid(c)
        g = torch.mul(a.expand_as(l), l)
        if self.normalize_attn:
            g = g.view(N,C,-1).sum(dim=2) # batch_sizexC
        else:
            g = F.adaptive_avg_pool2d(g, (1,1)).view(N,C)
        return c.view(N,1,W,H), g


def weights_init_xavierUniform(module):
    for m in module.modules():
        if isinstance(m, nn.Conv2d):
            nn.init.xavier_uniform_(m.weight, gain=np.sqrt(2))
            if m.bias is not None:
                nn.init.constant_(m.bias, 0)
        elif isinstance(m, nn.BatchNorm2d):
            nn.init.uniform_(m.weight, a=0, b=1)
            nn.init.constant_(m.bias, val=0.)
        elif isinstance(m, nn.Linear):
            nn.init.xavier_uniform_(m.weight, gain=np.sqrt(2))
            if m.bias is not None:
                nn.init.constant_(m.bias, val=0.)

class AttnNet(nn.Module):
    def __init__(self, im_size, dof, normalize_attn=True):
        super(AttnNet, self).__init__()
        
        # Convolutional blocks.
        self.conv_block1 = ConvBlock(4, 64, 2)
        self.conv_block2 = ConvBlock(64, 128, 2)
        self.conv_block3 = ConvBlock(128, 128, 2)
        self.conv_block4 = ConvBlock(128, 128, 2)
        self.conv_block5 = ConvBlock(128, 128, 2)
        self.dense = nn.Conv2d(in_channels=128, out_channels=128, kernel_size=int(im_size/utils.RESIZED_IMAGE_HEIGHT), padding=0, bias=True)

        self.attn1 = LinearAttentionBlock(in_features=128, normalize_attn=True)
        self.attn2 = LinearAttentionBlock(in_features=128, normalize_attn=True)
        self.attn3 = LinearAttentionBlock(in_features=128, normalize_attn=True)

        # Rnn.
        self.rnn = nn.LSTM(128*3, 10, 2, batch_first=True)

        # Velocity layer.
        self.vel = nn.Linear(in_features=10, out_features=dof, bias=True)

        # Initialize.
        weights_init_xavierUniform(self)

    def forward(self, x):
        b, t, c, h, w = x.size()
        x = x.view(b*t, c, h, w)

        # Feed forward.
        x = self.conv_block1(x)
        x = self.conv_block2(x)
        l1 = self.conv_block3(x)
        l2 = self.conv_block4(l1)
        l3 = self.conv_block5(l2)
        g = self.dense(l3) # batch_sizex512x1x1c
        # Pay attention.
        c1, g1 = self.attn1(l1, g)
        c2, g2 = self.attn2(l2, g)
        c3, g3 = self.attn3(l3, g)
        g = torch.cat((g1,g2,g3), dim=1) # batch_sizexC

        # Velocity layer.
        g = g.view(b, t, -1)
        g, _ = self.rnn(g)
        x = torch.tanh(self.vel(g)) # batch_sizexnum_classes

        return [x, c1, c2, c3]
