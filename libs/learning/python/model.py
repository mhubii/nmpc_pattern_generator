import torch
import torch.nn as nn
import torch.nn.functional as F


class RGBDCNN(nn.Module):
    """
        Convolutional neural net for RGBD images.
    """

    def __init__(self, input_shape, dof, batch_size):
        """
            Initialize the CNN.
        """

        super(RGBDCNN, self).__init__()

        self.batch_size = batch_size
        self.cnn = nn.Sequential(
            nn.Conv2d(4, 16, 5, 2),
            nn.Tanh(),
            nn.Conv2d(16, 32, 5, 2),
            nn.Tanh(),
            nn.Conv2d(32, 64, 3, 1),
            nn.Tanh(),
        )

        n = self._get_conv_output(self.cnn, input_shape)

        self.classification = nn.Sequential(
            nn.Linear(n, 16),
            nn.Tanh(),
            nn.Linear(16, 8),
            nn.Tanh(),
            nn.Linear(8, dof),
            nn.Tanh()
        )

        self.apply(self.weights_init_uniform)

    def _get_conv_output(self, net, shape):
        """
            Determine the dimension of the feature space.
        """

        # Unsqueeze to obtain 1x(shape) as dimensions.
        input = torch.rand(shape).unsqueeze(0)
        input = torch.autograd.Variable(input)
        output = net(input)
        n = output.numel()
        return n

    def forward(self, rgbd):
        """
            Forward rgb and depth image.
        """

        # Convolutional layers for feature extraction.
        out = self.cnn(rgbd)

        # Flatten.
        out = out.view(self.batch_size, int(out.numel()/self.batch_size))

        # Linear layers for classification.
        out = self.classification(out)

        return out

    def weights_init_uniform(self, m):
        classname = m.__class__.__name__
        # for every Linear layer in a model..
        if classname.find('Linear') != -1:
            # apply a uniform distribution to the weights and a bias=0
            m.weight.data.uniform_(0.0, 0.1)
            m.bias.data.fill_(0)


class RGBDCNNLSTM(nn.Module):
    """
        Convolutional neural net for sequence RGBD images.
    """

    def __init__(self, input_shape, dof):
        """
            Initialize the CNN LSTM.
        """

        super(RGBDCNNLSTM, self).__init__()

        self.conv1 = nn.Conv2d(4, 32, 5, 2)
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 64, 5, 2)
        self.bn2 = nn.BatchNorm2d(64)
        self.conv3 = nn.Conv2d(64, 128, 3, 1)
        self.bn3 = nn.BatchNorm2d(128)
        self.conv4 = nn.Conv2d(128, 128, 3, 1)
        self.bn4 = nn.BatchNorm2d(128)

        n = self._get_conv_output(input_shape)

        self.fc1 = nn.Linear(n, 256)
        self.fc2 = nn.Linear(256, 128)
        self.rnn = nn.LSTM(128, 64, 1, batch_first=True)
        self.fc3 = nn.Linear(64, dof)

        self.apply(self.weights_init_uniform)

    def _get_conv_output(self, shape):
        """
            Determine the dimension of the feature space.
        """

        # Unsqueeze to obtain 1x(shape) as dimensions.
        x = torch.rand(shape).unsqueeze(0)
        x = self.conv1(x)
        x = self.conv2(x)
        x = self.conv3(x)
        x = self.conv4(x)
        n = x.numel()
        return n

    def forward(self, x):
        """
            Forward rgb and depth image.
        """

        b, t, c, h, w = x.size()
        x = x.view(b*t, c, h, w)

        # Convolutional layers for feature extraction.
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.bn4(self.conv4(x)))

        # Flatten.
        x = x.view(b*t, -1)

        # Linear layers for classification.
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = x.view(b, t, -1)
        x, _ = self.rnn(x)
        x = self.fc3(x)

        return x

    def weights_init_uniform(self, m):
        classname = m.__class__.__name__
        # for every Linear layer in a model..
        if classname.find('Linear') != -1:
            # apply a uniform distribution to the weights and a bias=0
            m.weight.data.uniform_(0.0, 0.1)
            m.bias.data.fill_(0)


class GDCNN(nn.Module):
    """
        Convolutional neural net for gray depth images, e.g. for filtered red channel.
    """

    def __init__(self, input_shape, dof, batch_size):
        """
            Initialize the CNN.
        """

        super(GDCNN, self).__init__()

        self.batch_size = batch_size
        self.cnn = nn.Sequential(
            nn.Conv2d(2, 8, 5, 2),
            nn.ReLU(),
            nn.Conv2d(8, 16, 5, 2),
            nn.ReLU(),
            nn.Conv2d(16, 32, 3, 2),
            nn.ReLU(),
        )

        n = self._get_conv_output(self.cnn, input_shape)

        self.classification = nn.Sequential(
            nn.Linear(n, 16),
            nn.ReLU(),
            nn.Linear(16, 8),
            nn.ReLU(),
            nn.Linear(8, dof),
            nn.Tanh()
        )

    def _get_conv_output(self, net, shape):
        """
            Determine the dimension of the feature space.
        """

        # Unsqueeze to obtain 1x(shape) as dimensions.
        input = torch.rand(shape).unsqueeze(0)
        input = torch.autograd.Variable(input)
        output = net(input)
        n = output.numel()
        return n

    def forward(self, gd):
        """
            Forward rgb and depth image.
        """

        # Convolutional layers for feature extraction.
        out = self.cnn(gd)

        # Flatten.
        out = out.view(self.batch_size, int(out.numel()/self.batch_size))

        # Linear layers for classification.
        out = self.classification(out)

        return out
