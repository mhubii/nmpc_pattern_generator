import torch
import torch.nn as nn
import torch.nn.functional as F


class UNet(nn.Module):
    def __init__(self, input_shape, dof, batch_size):
        super(UNet, self).__init__()
        self.input_shape = input_shape
        self.batch_size = batch_size
        self.dof = dof
        self.inc = inconv(4, 64)
        self.down1 = down(64, 128)
        self.down2 = down(128, 128)
        self.up1 = up(256, 64)
        self.up2 = up(128, 64)
        self.outc = outconv(64, 4)
        n = self._get_conv_output(input_shape)
        self.fc1 = nn.Linear(n, 64)
        self.fc2 = nn.Linear(64, 32)

        # Rnn.
        self.rnn = nn.LSTM(32, 10, 1, batch_first=True)
        self.fc3 = nn.Linear(10, dof)

    def _get_conv_output(self, shape):
        """
            Determine the dimension of the feature space.
        """

        # Unsqueeze to obtain 1x(shape) as dimensions.
        input = torch.rand(shape).unsqueeze(0)
        input = torch.autograd.Variable(input)
        output = self.forward_skip(input)
        n = output.numel()
        return n

    def forward_skip(self, x):
        x1 = self.inc(x)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x = self.up1(x3, x2)
        x = self.up2(x, x1)
        x = self.outc(x)
        return torch.tanh(x)

    def forward(self, x):
        b, t, c, h, w = x.size()
        x = x.view(b*t, c, h, w)
        x = self.forward_skip(x)

        # Flatten.
        #x = x.view(b*t, int(x.numel()/(b*t)))
        x = x.view(b*t, -1)
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))

        x = x.view(b, t, -1)
        x, (h_c, h_c) = self.rnn(x)
        x = torch.tanh(self.fc3(x[:, -1, :]))
        
        return x


class GDUNet(nn.Module):
    def __init__(self, input_shape, dof, batch_size):
        super(GDUNet, self).__init__()
        self.input_shape = input_shape
        self.batch_size = batch_size
        self.dof = dof
        self.inc = inconv(2, 64)
        self.down1 = down(64, 128)
        self.down2 = down(128, 128)
        self.up1 = up(256, 64)
        self.up2 = up(128, 64)
        self.outc = outconv(64, 4)
        n = self._get_conv_output(input_shape)
        self.fc1 = nn.Linear(n, 64)
        self.fc2 = nn.Linear(64, 32)
        self.fc3 = nn.Linear(32, dof)

    def _get_conv_output(self, shape):
        """
            Determine the dimension of the feature space.
        """

        # Unsqueeze to obtain 1x(shape) as dimensions.
        input = torch.rand(shape).unsqueeze(0)
        input = torch.autograd.Variable(input)
        output = self.forward_skip(input)
        n = output.numel()
        return n

    def forward_skip(self, x):
        x1 = self.inc(x)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x = self.up1(x3, x2)
        x = self.up2(x, x1)
        x = self.outc(x)
        return torch.tanh(x)

    def forward(self, x):
        x = self.forward_skip(x)

        # Flatten.
        # x = x.view(self.batch_size, x.numel()/self.batch_size)
        x = x.view(self.batch_size, -1)
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)

        return x



class double_conv(nn.Module):
    '''(conv => BN => ReLU) * 2'''
    def __init__(self, in_ch, out_ch):
        super(double_conv, self).__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        x = self.conv(x)
        return x


class inconv(nn.Module):
    def __init__(self, in_ch, out_ch):
        super(inconv, self).__init__()
        self.conv = double_conv(in_ch, out_ch)

    def forward(self, x):
        x = self.conv(x)
        return x


class down(nn.Module):
    def __init__(self, in_ch, out_ch):
        super(down, self).__init__()
        self.mpconv = nn.Sequential(
            nn.MaxPool2d(2),
            double_conv(in_ch, out_ch)
        )

    def forward(self, x):
        x = self.mpconv(x)
        return x


class up(nn.Module):
    def __init__(self, in_ch, out_ch, bilinear=True):
        super(up, self).__init__()

        #  would be a nice idea if the upsampling could be learned too,
        #  but my machine do not have enough memory to handle all those weights
        #if bilinear:
        #    self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        #else:
        #    self.up = nn.ConvTranspose2d(in_ch//2, in_ch//2, 2, stride=2)

        self.conv = double_conv(in_ch, out_ch)

    def forward(self, x1, x2):
        #x1 = self.up(x1)
        x1 = nn.functional.interpolate(x1, scale_factor=2, mode='bilinear', align_corners=True)
        
        # input is CHW
        diffY = x2.size()[2] - x1.size()[2]
        diffX = x2.size()[3] - x1.size()[3]

        x1 = F.pad(x1, (diffX // 2, diffX - diffX//2,
                        diffY // 2, diffY - diffY//2))
        
        # for padding issues, see 
        # https://github.com/HaiyongJiang/U-Net-Pytorch-Unstructured-Buggy/commit/0e854509c2cea854e247a9c615f175f76fbb2e3a
        # https://github.com/xiaopeng-liao/Pytorch-UNet/commit/8ebac70e633bac59fc22bb5195e513d5832fb3bd

        x = torch.cat([x2, x1], dim=1)
        x = self.conv(x)
        return x


class outconv(nn.Module):
    def __init__(self, in_ch, out_ch):
        super(outconv, self).__init__()
        self.conv = nn.Conv2d(in_ch, out_ch, 1)

    def forward(self, x):
        x = self.conv(x)
        return x
