import torch
import torch.nn as nn


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
